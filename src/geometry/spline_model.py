import pdb

import numpy

import geometry.basis_spline

class SplineModel(object):
    """
    """
    def __init__(
        self,
        knots,
        order,
        coefficients=None,
        A=None,
        B=None,
        P=None,
        Q=None,
        R=None,
        boundary_knots=None):
        """
        """
        if coefficients is None:
            coefficients = numpy.zeros((len(knots) + order - 1,))
        self._spline = geometry.basis_spline.bSpline(
            knots,
            order,
            boundary_knots,
            coefficients)

        self._A = A
        self._B = B

        if P is None:
            P = numpy.diag(numpy.ones((len(self._spline),)))
        if not isinstance(P, numpy.ndarray):
            P *= numpy.diag(numpy.ones((len(self._spline),)))
        self._P = P

        if Q is None:
            Q = numpy.eye(len(self._spline))
        self._Q = Q

        self._R = R

    def time_update(self, dt, u=None, A=None, B=None, Q=None):
        """perform a time update on the model

        Arguments:
            dt: time step size
            u: optionally, the input
            A: optionally, the state transition matrix
            B: optionally, the input matrix
            Q: optionally, the process noise matrix to use

        Returns:
            no returns
        """
        if A is not None:
            self._spline.coords += A.dot(self._spline.coords) * dt
            self._P = A.dot(self._P).dot(A.T)
        elif self._A is not None:
            self._spline.coords += self._A.dot(self._spline.coords) * dt
            self._P = self._A.dot(self._P).dot(self._A.T)

        if Q is None:
            self._P += self._Q * dt
        else:
            if isinstance(Q, float):
                Q = numpy.eye(self._spline.coords.shape[0]) * Q
            self._P += Q * dt

        if u is not None:
            if B is not None:
                self._spline.coords += B.dot(u) * dt
            elif self._B is not None:
                self._spline.coords += self._B.dot(u) * dt

    def measurement(self, x, z, R=None):
        """perform a measurement update

        Arguments:
            x: location of the measurement
            z: measurement
            R: optionally, the measurement noise matrix to use

        Returns:
            no returns
        """
        vandermonde = numpy.array(self._spline.basis(x), ndmin=2)
        y = z - self._spline.eval(x)
        if R is None:
            s = vandermonde.dot(self._P).dot(vandermonde.T) + self._R
        else:
            s = vandermonde.dot(self._P).dot(vandermonde.T) + R
        K = self._P.dot(vandermonde.T) / s
        self._spline.coords += numpy.squeeze(K.dot(y))
        self._P = (numpy.eye(len(self._spline)) - K.dot(vandermonde)).dot(
            self._P)

    def evaluate(self, x):
        """Retrieve the model at a point

        Arguments:
            x: point to evaluate the model at

        Returns:
            y: model value at that point
        """
        return self._spline.eval(x)

    def basis(self, x):
        """Retrieve the basis at a point

        Arguments:
            x: the point to evaluate the model at

        Returns:
            N: the model basis at point x
        """
        return self._spline.basis(x)

    def sigma(self, x):
        """Retrieve the model uncertainty at a point

        Arguments:
            x: the point to retrieve uncertainty at

        Returns:
            sigma: model variance at this point
        """
        N = self._spline.basis(x)
        sigma = N.dot(self._P).dot(N.T)
        if isinstance(sigma, numpy.ndarray):
            return numpy.diag(sigma)
        else:
            return sigma

    @property
    def coefficients(self):
        """Getter for the spline coefficients

        Arguments:
            no arguments

        Returns:
            c: spline coefficients
        """
        return numpy.array(self._spline.coords)

    @coefficients.setter
    def coefficients(self, c):
        """Setter for spline coefficients

        Arguments:
            c: new spline coefficients. Must match current size of the
                coordinates

        Returns:
            no returns
        """
        assert self._spline.coords.shape == c.shape,\
            'input and spline size do not match'
        self._spline.coords = c

    @property
    def knots(self):
        """Getter for the spline knots

        Arguments:
            no arguments

        Returns:
            k: spline knot locations
        """
        return numpy.array(self._spline.knots)

    @property
    def order(self):
        """Getter for spline order

        Arguments:
            no arguments

        Returns:
            n: spline order
        """
        return self._spline.order

    @property
    def covariance(self):
        """Getter for the covaraince

        Arguments:
            no arguments

        Returns:
            P: model covarianace
        """
        return self._P

    @covariance.setter
    def covariance(self, P):
        """Set the covariance

        Arguments:
            P: model covaraiance. either a single value or a (n,n) array where
                n is the dimension of the spline coefficients. If it is a single
                value then this value will be set on the diagonal

        Returns:
            no returns
        """
        if not isinstance(P, numpy.ndarray):
            P = numpy.diag(numpy.ones((len(self._spline),))) * P
        self._P = P

