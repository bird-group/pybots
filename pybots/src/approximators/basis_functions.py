import pdb
import numpy

class GaussianRadialBasis(object):
    """
    """
    def __init__(self, sigma, x0=None):
        """Constructor

        Arguments:
            sigma: numpy (n,n) array giving the shape parameter. The dimension
                of the input vector should be equal to n
            x0: center point of the basis, numpy (1,n) array

        Returns:
            class instance
        """
        if x0 is None:
            x0 = numpy.zeros((1,sigma.shape[0]))
        self._sigma = sigma
        self._sinv = numpy.linalg.inv(sigma)
        self._x0 = x0

    def value(self, x):
        """Get the value of the radial basis at point X

        Arguments:
            x: vector specifying point to evaluate the basis at. (m,n) array
                where m is the number of vectors to evaluate simultaneously and
                n is the dimension of the vector

        Returns:
            f: basis function value at x
        """
        delta = x - self._x0
        anon_val_calculator = lambda xi: xi.dot(self._sinv).dot(xi)
        f = numpy.apply_along_axis(anon_val_calculator, 1, delta)
        return numpy.exp(-f)

    def gradient(self, x):
        """Get the gradient of the rbf at point x

        Arguments:
            x: vector specifying point to evaluate the basis at. (m,n) array
                where m is the number of vectors to evaluate simultaneously and
                n is the dimension of the vector

        Returns:
            grad_f: gradient of basis function value at x
        """
        grad = -2.0 * self.value(x) * (x - self._x0).dot(self._sigma)
        return grad

class Uniform(object):
    """A class for a uniform bias basis
    """
    def __init__(self):
        """Constructor

        Arguments:
            no arguments

        Returns:
            class instance
        """
        self._scale = 1.0

    def value(self, x):
        """Get the value of the uniform basis at point X

        Arguments:
            x: vector specifying point to evaluate the basis at. (m,n) array
                where m is the number of vectors to evaluate simultaneously and
                n is the dimension of the vector

        Returns:
            f: basis function value at x -- it is 1
        """
        if x.ndim > 1:
            return numpy.ones(x.shape[0])
        return 1.0

    def gradient(self, x):
        """Get the gradient of the rbf at point x

        Arguments:
            x: vector specifying point to evaluate the basis at. (m,n) array
                where m is the number of vectors to evaluate simultaneously and
                n is the dimension of the vector

        Returns:
            grad_f: gradient of basis function value at x
        """
        return numpy.zeros(x.shape)
