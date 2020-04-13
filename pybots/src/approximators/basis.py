import pdb

import itertools
import multiprocessing.pool

import numpy
import scipy.linalg
from numpy import float64 as precision

import approximators.basis_functions

def _evaluate_bases(bases, x):
    """Evaluate basis b at point x
    """
    return bases.value(x)
    C = numpy.stack([b.value(x) for b in bases])
    return C.T

class DirectProductBasisApproximator(object):
    """
    """
    def __init__(self, bases, w0=None, alpha=0.001):
        """

        Arguments:
            bases: iterable of (already constructed) basis functions to use
                there should be one entry per dimensions of the domain of the
                function to be modeled each entry should be an iterable of bases
            w0: optional, weight vector with dimension (prod(n_i),) where n_i
                are the number of basis functions in each dimension of the
                function domain. defaults to zeros
            alpha: optional, learning rate parameter, defaults 0.001

        Returns:
            class instance
        """
        self._bases = bases
        self._N = numpy.prod([len(b_i) for b_i in self._bases])
        if w0 is None:
            w0 = numpy.zeros((len(bases),))
        self._alpha = alpha
        if w0 is None:
            self._w = numpy.zeros((self._N, self._N))
        else:
            self._w = w0

    def C(self, x, parallel=True):
        """Compute the observation matrix

        Arguments:
            x: point of observation
            parallel: optional, compute C with parallel operations

        Returns:
            no returns
        """
        if parallel:
            return self.parC(x).T
        C = 1.0
        for xi, bases in zip(x.T, self._bases):
            C = numpy.kron(
                C, numpy.stack([b.value(x_i) for b in bases], dtype=precision))
        return C.T

    def parC(self, x):
        """This is a parallelized version of the calculation for the C matrix"""
        C = 1.0
        for bases in self._bases:
            with multiprocessing.pool.Pool() as pool:
                b = self._bases
                next_C = list(pool.starmap(
                    _evaluate_bases,
                    itertools.zip_longest(b, [], fillvalue=x)))
            C = numpy.kron(C, numpy.array(next_C, dtype=precision))
        return C

    def value(self, x):
        """Get the value of the basis field at point X

        Arguments:
            x: vector specifying point to evaluate the basis at. (m,n) array
                where m is the number of vectors to evaluate simultaneously and
                n is the dimension of the vector

        Returns:
            f: basis function value at x, summed from all basis functions
        """
        return numpy.dot(C, self._w)

    def gradient_x(self, x):
        """Get the gradient of the basis field at point x with respect to x

        Arguments:
            x: vector specifying point to evaluate the basis at. (m,n) array
                where m is the number of vectors to evaluate simultaneously and
                n is the dimension of the vector

        Returns:
            grad_f_x: gradient of function with respect to x
        """
        grad = []
        for w in self._w:
            grad = numpy.vstack(
                [b.gradient(x) * w_i for b, w_i in zip(self._bases, w)])
        return numpy.array(grad)

    def gradient_w(self, x):
        """Get the gradient of the basis field at point x with respect to w

        Arguments:
            x: vector specifying point to evaluate the basis at. (m,n) array
                where m is the number of vectors to evaluate simultaneously and
                n is the dimension of the vector

        Returns:
            grad_f_w: gradient of function with respect to w
        """
        return self.C(x)

    def sgd_update(self, x, observation, alpha=None):
        """Update using stochastic gradient descent rule

        Arguments:
            x: point observation was taken at
            observation: a measurement of the true value of the function
            alpha: optional, override learning rate

        Returns:
            None
        """
        if alpha is None:
            alpha = self._alpha
        self._w += alpha * numpy.squeeze(
            (observation - self.value(x)) * self.gradient_w(x).T)

class NDBasisApproximator(object):
    """
    """
    def __init__(self, bases, ndim=1, w0=None, alpha=0.001):
        """

        Arguments:
            bases: iterable of (already constructed) basis functions to use
                there should be one entry per dimensions of the domain of the
                function to be modeled each entry should be an iterable of bases
            ndim: the number of dimensions in the output of the approximation
            w0: optional, weight vector with dimension (prod(n_i),) where n_i
                are the number of basis functions in each dimension of the
                function domain. defaults to zeros
            alpha: optional, learning rate parameter, defaults 0.001

        Returns:
            class instance
        """
        self._bases = bases
        self._ndim = ndim
        self._N = len(bases)
        self._alpha = alpha
        if w0 is None:
            self._w = numpy.zeros((self._N * self._ndim,))
        else:
            self._w = w0

    def C(self, x, parallel=True):
        """Compute the observation matrix

        Arguments:
            x: point of observation
            parallel: optional, compute C with parallel operations

        Returns:
            no returns
        """
        if parallel:
            return self.parC(x)
        C = numpy.stack([b.value(x_i) for b in bases], dtype=precision)

        return scipy.linalg.block_diag(*[C.T,] * self._ndim)

    def parC(self, x):
        """This is a parallelized version of the calculation for the C matrix"""
        with multiprocessing.pool.Pool() as pool:
            b = self._bases
            next_C = list(pool.starmap(
                _evaluate_bases,
                itertools.zip_longest(b, [], fillvalue=x)))
        C = numpy.array(next_C, dtype=precision)
        return scipy.linalg.block_diag(*[C.T,] * self._ndim)

    def value(self, x):
        """Get the value of the basis field at point X

        Arguments:
            x: vector specifying point to evaluate the basis at. (m,n) array
                where m is the number of vectors to evaluate simultaneously and
                n is the dimension of the vector

        Returns:
            f: basis function value at x, summed from all basis functions
        """
        C = self.C(x)
        return numpy.reshape(numpy.dot(C, self._w), (self._ndim, -1)).T

    def gradient_x(self, x):
        """Get the gradient of the basis field at point x with respect to x

        Arguments:
            x: vector specifying point to evaluate the basis at. (m,n) array
                where m is the number of vectors to evaluate simultaneously and
                n is the dimension of the vector

        Returns:
            grad_f_x: gradient of function with respect to x
        """
        grad = []
        for w in self._w:
            grad = numpy.vstack(
                [b.gradient(x) * w_i for b, w_i in zip(self._bases, w)])
        return numpy.array(grad)

    def gradient_w(self, x):
        """Get the gradient of the basis field at point x with respect to w

        Arguments:
            x: vector specifying point to evaluate the basis at. (m,n) array
                where m is the number of vectors to evaluate simultaneously and
                n is the dimension of the vector

        Returns:
            grad_f_w: gradient of function with respect to w
        """
        return self.C(x)

    def sgd_update(self, x, observation, alpha=None):
        """Update using stochastic gradient descent rule

        Arguments:
            x: point observation was taken at
            observation: a measurement of the true value of the function
            alpha: optional, override learning rate

        Returns:
            None
        """
        if alpha is None:
            alpha = self._alpha
        self._w += alpha * numpy.squeeze(
            (observation - self.value(x)) * self.gradient_w(x).T)

    @property
    def w(self):
        return self._w

    @w.setter
    def w(self, new_w):
        self._w = new_w

class BasisKalmanFilter(object):
    """This is a super class...use one of the inherited classes
    """
    def __init__(
        self,
        x0=None, P0=None,
        R=numpy.eye(1), Q=None, A=None, B=None):
        """

        Arguments:
            bases: iterable of (already constructed) basis functions to use
                there should be one entry per dimensions of the domain of the
                function to be modeled each entry should be an iterable of bases
            x0: optional, weight vector with dimension (prod(n_i),) where n_i
                are the number of basis functions in each dimension of the
                function domain. defaults to zeros
            P0: numpy (prod(n_i), prod(n_i)) matrix of covariance. It can be
                specified with a scalar value which is put on the diagonal
            R: optional, measurement noise, defaults to 1
            Q: optional, process noise, defaults to 0
            A: optional, state transition matrix, defaults to I
            B: optional, input matrix, defaults to zeros

        Returns:
            class instance
        """
        n_weights = self._N * self._ndim
        if x0 is None:
            self._x = numpy.zeros((n_weights))
        elif not isinstance(x0, numpy.ndarray):
            self._x = numpy.ones((n_weights,)) * x0
        else:
            self._x = x0

        if P0 is None:
            self._P = numpy.eye(n_weights)
        elif not isinstance(P0, numpy.ndarray):
            self._P = numpy.eye(n_weights) * P0
        else:
            self._P = P0

        self._R = R
        if Q is None:
            Q = numpy.zeros(self._P.shape)
        self._Q = Q
        if A is None:
            A = numpy.eye(n_weights)
        self._A = A
        if B is None:
            B = numpy.zeros((n_weights))
        self._B = B

    def C(self, x, parallel=True):
        """Compute the observation matrix

        Arguments:
            x: point of observation
            parallel: optional, compute C with parallel operations

        Returns:
            no returns
        """
        C_i = self._approximator.C(x, parallel)
        C = scipy.linalg.block_diag(*[C_i,]*self._ndim)
        return C

    def value(self, x):
        """Get the value of the basis field at point X

        Arguments:
            x: vector specifying point to evaluate the basis at. (m,n) array
                where m is the number of vectors to evaluate simultaneously and
                n is the dimension of the vector

        Returns:
            f: basis function value at x, summed from all basis functions
        """
        C = self.C(x)
        return numpy.reshape(numpy.dot(C, self._x), (self._ndim, -1)).T

    def sigma(self, x):
        C = self.C(x)
        return C.dot(self._P).dot(C.T)

    def gradient_w(self, x):
        """Get the gradient of the basis field at point x with respect to w

        Arguments:
            x: vector specifying point to evaluate the basis at. (m,n) array
                where m is the number of vectors to evaluate simultaneously and
                n is the dimension of the vector

        Returns:
            grad_f_w: gradient of function with respect to w
        """
        return self.C(x)

    def time_update(self, u=None, A=None, B=None, Q=None):
        """do a time update

        Arguments:
            u: (p,) input vector
            A: optional, (m, m) state transition matrix
            B: optional, (m, p) input matrix.
            Q: optional, (m, m) process noise matrix

        Returns:
            None
        """
        if u is None:
            u = numpy.zeros((1,), dtype=precision)
        if A is None:
            A = self._A
        if B is None:
            B = self._B
        if Q is None:
            Q = self._Q
        self._x = A.dot(self._x) + B.dot(u)
        self._P = A.T.dot(self._P).dot(A) + Q

    def measurement_update(self, x, z, R=None):
        """do a measurement update

        Arguments:
            x: point the measurement was taken
            z: scalar measurement
            R: optional, scalar measurement uncertainty

        Returns:
            No returns
        """
        if R is None:
            R = self._R

        if x.ndim == 1:
            x = numpy.array(x, ndmin=2)

        C = self.C(x)
        y = z - C.dot(self._x)
        S = C.dot(self._P).dot(C.T) + R
        K = numpy.linalg.solve(S.T, self._P.dot(C.T).T).T
        self._x += K.dot(y)
        self._P = (numpy.eye(self._N * self._ndim) - K.dot(C)).dot(self._P)

class DirectProductBasisKalmanFilter(BasisKalmanFilter):
    """
    """
    def __init__(
        self,
        bases, ndim=1, x0=None, P0=None,
        R=numpy.eye(1), Q=None, A=None, B=None):
        """

        Arguments:
            bases: iterable of (already constructed) basis functions to use
                there should be one entry per dimensions of the domain of the
                function to be modeled each entry should be an iterable of bases
            x0: optional, weight vector with dimension (prod(n_i),) where n_i
                are the number of basis functions in each dimension of the
                function domain. defaults to zeros
            P0: numpy (prod(n_i), prod(n_i)) matrix of covariance. It can be
                specified with a scalar value which is put on the diagonal
            R: optional, measurement noise, defaults to 1
            Q: optional, process noise, defaults to 0
            A: optional, state transition matrix, defaults to I
            B: optional, input matrix, defaults to zeros

        Returns:
            class instance
        """
        self._approximator = DirectProductBasisApproximator(bases)
        self._N = numpy.prod([len(b_i) for b_i in bases])
        self._ndim = ndim

        super(DirectProductBasisKalmanFilter, self).__init__(x0, P0, R, Q, A, B)

class NDBasisKalmanFilter(BasisKalmanFilter):
    """
    """
    def __init__(
        self,
        bases, ndim=1, x0=None, P0=None,
        R=numpy.eye(1), Q=None, A=None, B=None):
        """

        Arguments:
            bases: iterable of (already constructed) basis functions to use
                there should be one entry per dimensions of the domain of the
                function to be modeled each entry should be an iterable of bases
            ndim: the number of dimensions in the output of the approximation
            x0: optional, weight vector with dimension (prod(n_i),) where n_i
                are the number of basis functions in each dimension of the
                function domain. defaults to zeros
            P0: numpy (prod(n_i), prod(n_i)) matrix of covariance. It can be
                specified with a scalar value which is put on the diagonal
            R: optional, measurement noise, defaults to 1
            Q: optional, process noise, defaults to 0
            A: optional, state transition matrix, defaults to I
            B: optional, input matrix, defaults to zeros

        Returns:
            class instance
        """
        self._approximator = NDBasisApproximator(bases)
        self._N = len(bases)
        self._ndim = ndim

        super(NDBasisKalmanFilter, self).__init__(x0, P0, R, Q, A, B)