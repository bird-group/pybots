# -*- coding: utf-8 -*-
"""
Created on Mon Jun  8 11:48:05 2015

@author: Nate
"""
import pdb

import numpy as np
import numpy.linalg as la
import scipy.linalg as spla

class ExtendedKalmanFilter(object):
    """Class for a general Extended Kalman Filter."""
    pass

class UnscentedKalmanFilter(object):
    """Class for a general Unscented Kalman Filter."""
    # TODO: allow an array of measurement functions (and R matrices) to be
    #       passed in at initialization
    # TODO: in the measurement update method, provide an index for the
    #       measurement model that should be used
    # TODO: kalman filters should inherit from a common base class as a number
    #       of the methods are the same
    def __init__(self, sys_dyn, meas_dyn, Q, R, P0, x0, sys_lim=None):
        """Initialize the Unscentd Kalman Filter

        Args:
            sys_dyn: handle to a fucntion containing system dynamics.
            meas_dyn: heandle to a function containing the measurement model
            Q: (nxn) numpy array containing the system noise matrix
            R: (mxm) numpy array containing the measurement noise matrix
            P0: (nxn) numpy array containing the initial state covariance
            x0: (nx1) numpy array containing the initial system state
            sys_lim: optional handle to a function that limits the system state
        """
        # set estimator size (the length of the input vector)
        self.n = len(x0)
        # set weight matrices
        beta = 2.0
        alpha = 1.0
        # initialize weight array
        self.wm = (np.ones([2*self.n +1, 1], dtype=float) *
                    1.0/(2.0*self.n*alpha**2.0))
        self.wm[0] = (alpha**2.0 - 1.0)/(alpha**2.0)
        # initialize weight matrix
        self.Wc = (np.eye((2*self.n + 1), dtype=float) *
                    1.0/(2.0*self.n*alpha**2.0))
        self.Wc[0, 0] = ((alpha**2.0 - 1.0)/(alpha**2.0) +
                    (1.0 - alpha**2.0 + beta))
        # sigma point spacing
        self.eta = alpha*np.sqrt(self.n)
        # set estimator states and noise matrices
        self.xhat = None
        self.x0 = None
        self.P = None
        self.P0 = None
        self.X = None
        self.Q = None
        self.R = None
        self.set_state(x0, P0)
        self.set_noise(R, Q)
        # set non-linear state functions
        # system dynamics must accept arguments (x, u, z)
        self.system_dynamics = sys_dyn
        # measurement dynamics must accept arguments (x, u)
        self.measurement_dynamics = meas_dyn
        # system limits must accept arguments (x)
        self.system_limits = sys_lim

    def set_state(self, state=None, covariance=None):
        """If this method is called with no arguments, it resets the state and
        covariance matrices. At the end of this method, an initial set of sigma
        points is computed.

        Args:
            state: either a list or a (nx1) numpy array representing the
                initial state of the system
            covariance:
        """
        if state is not None:
            # set the current state estimate to the input value
            if type(state) is list:
                self.xhat = np.array([state]).T
            else:
                self.xhat = state
            # set the fallback state to the input value
            self.set_initial_state(self.xhat)
        else:
            # reset state estimate
            self.xhat = self.x0

        if covariance is not None:
            # set the current covariance matrix and fallback to input value
            self.P = covariance
            self.P0 = covariance
        else:
            # reset the cavariance matrix
            self.P = self.P0
        # set the initial set of sigma points
        self.X = self._sigma_points()
        # TODO: if getting sigma points fails, warn user

    def set_initial_state(self, state):
        """Method is used to set the initial filter state which is also used as
        the "fallback" state in the event that something bad happens to the
        filter.

        Args:
            state: either a list or a (nx1) numpy array representing the
                initial state of the system
        """
        if type(state) is list:
            self.x0 = np.array([state]).T
        else:
            self.x0 = state

    def set_noise(self, meas_noise, system_noise=None):
        """Method sets the noise covariance matrices for the system. If this is
        called without a second argument, it will change the measurement noise
        matrix only. This can be useful if the measurement noise is state
        dependant (ex. noise changes when the motor is turned on).

        Args:
            meas_noise: a (mxm) where m is the dimension of the measurement
                vector matrix representing the R matrix for the system.
            system_noise: an (nxn) numpy array representing the Q matrix for
                the system
        """
        # set the measurement noise covariance matrix
        self.R = meas_noise
        # if a new system noise matrix was provided, set it
        if system_noise is not None:
            self.Q = system_noise

    def measurement_update(
        self,
        yt,
        ut=None,
        measurement_dynamics=None,
        R=None):
        """Method incorporates a measurement into the filter state. This method
        does the traditional UKF measurement update. It will also check for
        states exceeding limits before exiting.

        Args:
            yt: numpy array containing the measurements
            ut: optional numpy array containing the system inputs
            measurement_dynamics: optional function handle to the measurement
                equation to use, should accept (state, input).
            R: optional matrix of measurement covariance
        """
        if R is None:
            R = self.R
        if measurement_dynamics is None:
            measurement_dynamics = self.measurement_dynamics
        # TODO: add an additional input indicating which measuremnts are to be
        # used at the current time. This allows us to ignore measurements that
        # we know to be bad.
        # compute the expected measurements from sigma points
        Chi = self._sigma_points()
        Y = np.zeros([len(yt), (2*self.n+1)], dtype=float)
        for i in range(2*self.n+1):
            #if we have inputs, pass them in
            if ut is not None:
                Y[:,i] = measurement_dynamics(Chi[:, i], ut)
                #Y[:,i] = measurement_dynamics(self.X[:, i], ut)
            else:
                Y[:,i] = measurement_dynamics(Chi[:, i])
                #Y[:,i] = measurement_dynamics(self.X[:, i])
        yhat = np.dot(Y, self.wm)
        # compute innovation statistics
        # TODO: Chi-squared test here as suggested by Chambers et al 2014
        yhatbig = np.tile(yhat, (1, (2*self.n+1)))
        YY = Y - yhatbig
        Pyy = np.dot(YY, np.dot(self.Wc, YY.T)) + R
        xhatbig = np.tile(self.xhat, (1, (2*self.n+1)))
        XX = Chi - xhatbig
        #XX = self.X - xhatbig
        Pxy = np.dot(XX, np.dot(self.Wc, YY.T))
        # compute a gain array
        # TODO: this could be a danger spot...
        #K = np.dot(Pxy, np.linalg.inv(Pyy))
        K = np.linalg.solve(Pyy, Pxy.T).T
        # update state estimate
        self.xhat = self.xhat + np.dot(K, (yt - yhat))
        # project xhat onto contrained space if a limit function s given
        if self.system_limits is not None:
            self.xhat = self.system_limits(self.xhat)
        # update covarience
        self.P = self.P - np.dot(K, np.dot(Pyy, K.T))

    def time_update(self, ut, zt, dt):
        """Method for the time update of a UKF.

        Args:
            ut: (1xm) numpy array representing the current system input
            zt: (?x1) numpy array representing additional inputs
            dt: float timestep

        Returns:
            No returns
        """
        # recompute the current set of sigma points
        Chi = self._sigma_points()
        # propogate sigma points through system dynamics
        for i in range(2*self.n+1):
            # project Chi onto contrained space if a limit function s given
            if self.system_limits is not None:
                Chi[:, i] = self.system_limits(Chi[:, i])
            #Chi[:,i] = self._rk4( Chi[:,i], ut, zt, dt )
            Chi[:, i] = self._euler(Chi[:, i], ut, zt, dt)
            # project Chi onto contrained space if a limit function s given
            if self.system_limits is not None:
                Chi[:, i] = self.system_limits(Chi[:, i])
        # save the updated sigma points to use in measurement update
        self.X = Chi
        # form updated sigma points compute a new xhat
        self.xhat = np.dot(Chi, self.wm)
        # compute updated covarience
        xhatbig = np.tile(self.xhat, (1, (2*self.n+1)))
        XX = Chi - xhatbig
        self.P = np.dot(XX, np.dot(self.Wc, XX.T)) + self.Q

    def _sigma_points(self):
        """Method computes the current set of sigma points based on the current
        state estimate and the associated covariance. This method also checks
        that the state and covariance are valid and will reset them otherwise.

        Args:
            No arguments

        Returns:
            An (n x 2n+1) numpy array of sigma points
        """
        # find sqrtP, but check for non positive definite matrices
        try:
            sqrtP = np.linalg.cholesky(self.P) # returns lower
        except:
            # we probably settled a bit too far, reset
            # TODO: be careful about different types of exceptions
            sqrtP = np.linalg.cholesky(self.P0) # returns lower of initial covarience matrix
        # check for a junk state, if so, reset
        if np.isnan(np.sum(self.xhat)):
            self.xhat = self.x0
            sqrtP = np.linalg.cholesky(self.P0)
        # compute sigma points
        Chi = np.tile(self.xhat, (1, (2*self.n+1)))
        Chi = Chi + np.concatenate(
            (
                np.zeros([self.n, 1], dtype=float),
                1.0 * self.eta * sqrtP,
                -1.0 * self.eta * sqrtP),
            axis=1)
        # return the sigma point array
        return Chi

    def _euler(self, X, U, zt, dt):
        """Method integrates the system state using Euler integration of the
        provided system model.

        Args:
            X: (nx1) numpy array representing the current state
            U: numpy array representing the current system input
            zt: numpy array representing other inputs
            dt: float indicating the integration timestep (sec)

        Returns:
            X: (nx1) numpy array representing the state advanced by dt
        """
        X = X + dt * self.system_dynamics(X, U, zt)
        return X

    def _rk4(self, X, U, zt, dt):
        """Method integrates the state forward using Fourth Order Runge Kutta
        integration and the provided system model. Note that this seems to be
        uber slow."""
        k1 = dt * self.system_dynamics(X, U, zt)
        k2 = dt * self.system_dynamics(X + k1/2.0, U, zt)
        k3 = dt * self.system_dynamics(X + k2/2.0, U, zt)
        k4 = dt * self.system_dynamics(X + k3, U, zt)
        X = X + 1.0/6.0*(k1 + 2.0*k2 + 2.0*k3 + k4)
        return X

    def return_state(self):
        """Method returns the current filter state as a list.

        Args:
            No input arguments

        Returns:
            a list holding the current filter state"""
        return self.xhat.squeeze().tolist()

    def return_var(self, idxes):
        """Method returns the diagonal of the covariance matrix flattened into
        a list for the idxes of interest.

        Args:
            idxes: a list of relevant indices

        Returns:
            a list containign the relevant entries on the diagonal of the
                covariance matrix
        """
        return self.P.diagonal()[idxes].tolist()

    def return_covar(self, idxes):
        """Method returns the covariance matrix entries corresponding to the
        requested indices as a list in row-major format.

        Args:
            idxes: a list of the relevant indices

        Returns:
            a list containing the relevant portion of the covariance matrix in
                row-major format
        """
        out = []
        for i in idxes:
            for j in idxes:
                out.append(self.P[i, j])
        return out

    def return_trace(self):
        """ Method returns the trace of the covariance matrix. this can be used
        for checking the health of the estimator.

        Args:
            No arguments

        Returns:
            trace of the covariance matrix as a float
        """
        return self.P.trace()
