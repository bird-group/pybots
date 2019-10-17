"""Routines for performing transformations on covariances

This may not stay here but I don't have a better place to put it for now.
"""
import pdb
import numpy

def linearization(X, Jfun, P):
    """Transform a covariance matrix via linearization

    Arguments:
        X: the point to linearize about, (n,) numpy array
        Jfun: function which takes the state and returns the (n x n) Jacobian of
            the function, f, which we want to transform the covariance by. It
            should return an (n x n) matrix
                df1dx1 df1dx2 ... df1dxn
                df2dx1 df2dx2 ... df2dxn
                 ...    ...   ...  ...
                dfndx1 dfndx2 ... dfndxn
        P: covariance matrix to transform

    Returns:
        P_prime: transformed covariance matrix
    """
    A = Jfun(X)
    P_prime = A.dot(P.dot(A.T))
    return P_prime

def unscented(fun, X, P, alpha=1.0, beta=2.0, kappa=1.0):
    """Transform a covariance matrix via unscented tranform

    This can raise a LinAlgError("Matrix is not positive definite") if the
    matrix P is not positive definite. The outer program should implement a try
    catch for this condition

    Arguments:
        X: the point to use as the anchor point for our transformation
        fun: transformation function which relates X' to X (X' = fun(X))
        P: covariance to transform
        alpha: parameter of the tranform
        beta: parameter of the transform

    Returns:
        P_prime: transformed covariance matrix
    """
    n = max(X.shape)
    lam = numpy.power(alpha, 2.0) * (n + kappa) - n
    wm = numpy.ones((2 * n + 1, 1)) / 2.0 / (n + lam)
    wm[0] = lam / (n + lam)
    # initialize weight matrix
    Wc = numpy.diag(wm[:,0])
    Wc[0, 0] = lam / (n + lam) + (1 - numpy.power(alpha, 2.0) + beta)

    eta = numpy.sqrt(n + lam)

    sqrtP = numpy.linalg.cholesky(P)

    Chi = numpy.tile(X, (2 * n + 1, 1))
    plus = numpy.concatenate(
        (numpy.zeros((n, 1)), eta * sqrtP, -eta * sqrtP), axis=1)
    Chi += plus.T

    # propogate sigma points through system dynamics
    Chi_prime = numpy.vstack((fun(Xi) for Xi in Chi))
    # form updated sigma points compute a new xhat
    x_prime = wm.T.dot(Chi_prime)
    # compute updated covarience
    x_prime_big = numpy.tile(x_prime, (2 * n + 1, 1))
    XX = Chi_prime - x_prime_big
    P_prime = XX.T.dot(Wc.dot(XX))
    return P_prime
