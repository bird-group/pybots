import numpy as np
import copy

import pdb

def lls(objective_function=None, X=0, Z=0, theta_0=None, convergence_tol=1e-4,
    iter_limit=100, theta_limit=None, relaxation_factor=1.0, scale_vector=None):
    """
    Does a linearized least squares to iterate for the best solution to a
    problem defined by:
        z=f(x, theta)
    Given samples of x and z, the function attempts to find the parameters theta

    Arguments:
        obj_fcn: should return Z and the Jacobian of Z with theta as a function
            of X and theta. For a system with n states, m outputs, p parameters,
            and q samples, X will be (n,q) Z will be (m,q) J_Z will be (m,p)
            (Z_hat, J_Z) = obj_fcn(X, theta)
        X: samples of X, the function state
        Z: corresponding samples of Z, the function output
        theta_0: an initial guess for theta, must be specified so we know the
            size of theta
        convergence_tol: iteration is terminated when the residual from the
            newton-raphson solver drops below this value
        iter_limit: iteration is alternately terminated when this number of
            iterations is reached
        theta_limit: (lower_lim, upper_lim), tuple with arrays same size as
            theta giving the upper and lower limits.
        relaxation_factor: the increment between iterations is multiplied by
            this factor to slow down or speed up the convergence
        scale_vector : a vector used to scale the components of theta, used in
            computing the norm and in adjusting the relative magnitude of the
            increment in theta during each iteration. The vector is made into a
            diagonal matrix SCALE. The norm is computed:
            sqrt(theta*SCALE'*theta') and the parameter increment is computed
            SCALE*theta. This defaults to an identity matrix

    Returns:
        (theta, residual, iterations)
        theta: the solved value for theta
        final_tol: the N-R residual when iteration was terminated
        iterations: the number of iterations at termination
    """
    assert hasattr(objective_function, '__call__'), 'objective function must be\
        a valid function'
    assert theta_0 is not None, 'must specify initial guess for theta'

    if theta_limit is None:
        theta_limit = (np.full(theta_0.shape, -np.inf),
            np.full(theta_0.shape, np.inf))

    if scale_vector is None:
        scale_matrix = np.identity(theta_0.shape[0])
        scale_matrix_squared = scale_matrix
    else:
        assert scale_vector.shape == theta_0.shape, 'tolerance scale must have the\
            same dimension as the parameter vector'
        scale_matrix = np.diag(scale_vector)
        scale_matrix_squared = scale_matrix*scale_matrix

    delta = 100
    iter_count = 0

    delta_theta = np.zeros(theta_0.shape)

    theta = copy.deepcopy(theta_0)

    while delta > convergence_tol and iter_count < iter_limit:
        iter_count += 1

        Z_hat, J_Z = objective_function(X, theta)
        delta_Z = Z - Z_hat

        delta_theta = np.linalg.lstsq(J_Z, delta_Z)[0]

        delta = np.sqrt(
            delta_theta.dot(scale_matrix_squared).dot(delta_theta.T))

        theta += scale_matrix.dot(delta_theta) * relaxation_factor

        theta = np.maximum(theta, theta_limit[0])
        theta = np.minimum(theta, theta_limit[1])

    return (theta, delta_theta, iter_count)
