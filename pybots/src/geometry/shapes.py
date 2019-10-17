import pdb

import numpy
import scipy.integrate
import scipy.optimize

def hypotrochoid(R, r, d, theta, direction=1):
    """Get the coordinates of a point on a hypotrochoid

    Arguments:
        R: radius of fixed circle in hypotrochoid
        r: radius of rotated circle in hypotrochoid
        d: offset of traced point
        theta: path parameter, can be numpy array

    Returns:
        X: numpy (2,) array giving points on hypotrochoid. if theta was a numpy
            array with n elements then this will be (n,2)
    """
    x_h = (
        (R - r) * numpy.cos(theta * direction) +
        d * numpy.cos((R - r) / r * theta * direction))
    y_h = (
        (R - r) * numpy.sin(theta * direction) -
        d * numpy.sin((R - r) / r * theta * direction))

    if isinstance(theta, numpy.ndarray):
        return numpy.vstack((x_h, y_h)).T

    return numpy.array([x_h, y_h])

class Hypotrochoid(object):
    """Implement the hypotrochoid as a class

    This will have evaluators and derivatives
    """
    def __init__(self, R, r, d, direction=1, X0=numpy.zeros((2,))):
        """Constructor

        Arguments:
            R: radius of fixed circle in hypotrochoid
            r: radius of rotated circle in hypotrochoid
            d: offset of traced point
            direction: which way does it go around. Defaults to 1 which gives
                a positive yaw rate (NED coordinates)
            X0: optionally, an offset for the Hypotrochoid

        Returns:
            class instance
        """
        self._R = R
        self._r = r
        self._d = d
        self._X0 = X0
        self._direction = direction

    def X(self, theta):
        """get the location on the hypotrochoid given a parameter

        Arguments:
            theta: the hypotrochoid parameter

        Returns:
            X: location on the hypotrochoid
        """
        X = hypotrochoid(
            self._R, self._r, self._d, theta, self._direction) + self._X0
        return X

    def dX_dtheta(self, theta):
        """Get a derivative of the path

        Arguments:
            theta: the hypotrochoid parameter

        Returns:
            dXtheta: location on the hypotrochoid
        """
        delta_r = self._R - self._r

        dx_dtheta = (
            -delta_r * numpy.sin(theta * self._direction) -
            self._d * delta_r / self._r * numpy.sin(
                delta_r / self._r * theta * self._direction))
        dy_dtheta = (
            delta_r * numpy.cos(theta * self._direction) -
            self._d * delta_r / self._r * numpy.cos(
                delta_r / self._r * theta * self._direction))

        if isinstance(theta, numpy.ndarray):
            return numpy.vstack((dx_dtheta, dy_dtheta)).T

        return numpy.array([dx_dtheta, dy_dtheta])

    def d2X_dtheta2(self, theta):
        """Get second derivative of the path

        Arguments:
            theta: the hypotrochoid parameter

        Returns:
            dX2theta2: location on the hypotrochoid
        """
        delta_r = self._R - self._r

        dx_dtheta = (
            -delta_r * numpy.cos(theta * self._direction) -
            (
                self._d *
                numpy.power(delta_r / self._r, 2.0) *
                numpy.cos(delta_r / self._r * theta * self._direction)))
        dy_dtheta = (
            -delta_r * numpy.sin(theta * self._direction) +
            (
                self._d *
                numpy.power(delta_r / self._r, 2.0) *
                numpy.sin(delta_r / self._r * theta * self._direction)))

        if isinstance(theta, numpy.ndarray):
            return numpy.vstack((dx_dtheta, dy_dtheta)).T

        return numpy.array([dx_dtheta, dy_dtheta])

    def d3X_dtheta3(self, theta):
        """Get third derivative of the path

        Arguments:
            theta: the hypotrochoid parameter

        Returns:
            dXrtheta3: location on the hypotrochoid
        """
        delta_r = self._R - self._r

        dx_dtheta = (
            delta_r * numpy.sin(theta * self._direction) -
            (
                self._d *
                numpy.power(delta_r / self._r, 3.0) *
                numpy.sin(delta_r / self._r * theta * self._direction)))
        dy_dtheta = (
            delta_r * numpy.cos(theta * self._direction) -
            (
                self._d *
                numpy.power(delta_r / self._r, 3.0) *
                numpy.cos(delta_r / self._r * theta * self._direction)))

        if isinstance(theta, numpy.ndarray):
            return numpy.vstack((dx_dtheta, dy_dtheta)).T

        return numpy.array([dx_dtheta, dy_dtheta])

    def ds_dtheta(self, theta):
        """Get the derivative of the path length wrt theta

        Arguments:
            theta: hypotrochoid location parameter

        Returns:
            ds_dtheta: derivative of path length with respect to theta
        """
        ds_dtheta = (self._R - self._r) * numpy.sqrt(
            1.0 +
            numpy.power(self._d / self._r, 2.0) -
            self._d / self._r * numpy.cos(
                self._R / self._r * theta * self._direction))

        return ds_dtheta * self._direction

    def d2s_dtheta2(self, theta):
        """Get the second derivative of the path wrt theta

        Arguments:
            theta: hypotrochoid location parameter

        Returns:
            d2s_dtheta2: second derivative of path length wrt theta
        """

        dr = self._d / self._r
        d2s_dtheta2 = (
            (self._R - self._r) *
            (dr * self._R / self._r * numpy.sin(
                self._R / self._r * theta * self._direction)) /
            numpy.sqrt(
                1 +
                numpy.power(dr, 2.0) -
                dr * numpy.cos(self._R / self._r * theta * self._direction)))
        return d2s_dtheta2

    def path_length(self, theta_0, theta_1):
        """Integrate the path

        Arguments:
            theta_0: start point
            theta_1: end point

        Returns:
            L: integrated path length
        """
        L = scipy.integrate.quad(
            self.ds_dtheta,
            theta_0 * self._direction,
            theta_1 * self._direction)
        return L[0]

    def path_curvature(self, theta):
        """Compute the radius of curvature of the path

        Arguments:
            theta: hypotrochoid location parameter

        Returns:
            R: radius of curvature
        """

        # the (signed) path curvature can be computed
        #
        #        x' y'' - y' x''
        # k =  -------------------
        #      (x'^2 + y'^2)^(3/2)
        #
        # (note that this is the inverse of the radius of curvature)

        Xp = self.dX_dtheta(theta)
        Xpp = self.d2X_dtheta2(theta)

        if isinstance(theta, numpy.ndarray):
            xp = Xp[:, 0]
            yp = Xp[:, 1]
            xpp = Xpp[:, 0]
            ypp = Xpp[:, 1]
        else:
            xp = Xp[0]
            yp = Xp[1]
            xpp = Xpp[0]
            ypp = Xpp[1]

        k = (
            (xp * ypp - yp * xpp) /
            numpy.power(xp * xp + yp * yp, 3.0 / 2.0))

        return 1.0 / k

    def path_curvature_derivative(self, theta):
        """Compute the derivative of the curvature

        Arguments:
            theta: hypotrochoid location parameter

        Returns:
            dR_dtheta: radius of curvature
        """

        # From the path curvature you can compute the derivative wrt theta
        #
        # let
        #       s' = sqrt(x'^2 + y'^2)
        #       n = x'y'' - y'x''
        #
        #       so that R = s'^3 / n
        #
        #               3s''s'^2n - n's'^3
        # dR_dtheta =   -------------------
        #                       n^2
        #

        Xp = self.dX_dtheta(theta)
        Xpp = self.d2X_dtheta2(theta)
        Xppp = self.d3X_dtheta3(theta)

        if isinstance(theta, numpy.ndarray):
            xp = Xp[:, 0]
            yp = Xp[:, 1]
            xpp = Xpp[:, 0]
            ypp = Xpp[:, 1]
            xppp = Xppp[:, 0]
            yppp = Xppp[:, 1]
        else:
            xp = Xp[0]
            yp = Xp[1]
            xpp = Xpp[0]
            ypp = Xpp[1]
            xppp = Xppp[0]
            yppp = Xppp[1]

        sp = numpy.sqrt(self.ds_dtheta(theta))
        n = xp * ypp - yp * xpp
        spp = (xpp * xp + ypp * yp) / sp
        np = xp * yppp -xppp *yp

        dR_dtheta = (
            (3.0 * spp * numpy.power(sp, 2.0) * n - np * numpy.power(sp, 3.0)) /
            numpy.power(n, 2.0))
        return dR_dtheta

    def theta(self, X, theta_0):
        """Compute the parameter for the nearest point on the path

        Finds the parameter, theta, which computes the point on the path
        nearest to the specified position. The search for theta is limited to
            theta_0 - pi / n_petals < theta < theta_0 + pi / n_petals
        this ensures that we don't end up on the wrong branch. It does however
        require that we're reasonably close to the path.

        Arguments:
            X: position to search
            theta_0: initial parameter

        Returns:
            theta: best fit path parameter
        """

        def objective(theta, X, hypotrochoid):
            """The objective to minimize

            Arguments:
                X: position to test, numpy (2,) array
                theta_0: search near this value of the path parameter

            Returns:
                l: norm of offset between candidate and desired location
            """
            return numpy.linalg.norm(hypotrochoid.X(theta) - X)

        bound = self._d * 2.0
        n = 1 + bound / 2.0 / self._r

        args = (X, self)
        bounds = numpy.array([-1, 1]) * numpy.pi / n + theta_0
        theta = scipy.optimize.minimize_scalar(
            objective, bounds, args=args, bounds=bounds, method='Bounded')
        return theta.x

