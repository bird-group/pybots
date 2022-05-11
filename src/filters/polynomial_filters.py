# -*- coding: utf-8 -*-
"""
Created on Fri Mar 20 12:20:05 2015

@author: Nate
"""
import numpy as np
import numpy.linalg as la

class PolynomialFilter():
    """Class codes a Chebychev polynomial filter."""
    def __init__(self, filter_order, window_size, pct_delay=0.0):
        # pct_delay [0,1] is the percent of the window size delay that is allowed. 0.5 should yeild maximum smoothing, 0.0 should yeild minimum delay
        # This will handle up to a 11th order polynomial fit but only return 3 derivatives
        self.window_size = window_size
        self.filter_order = filter_order
        self.frame = np.zeros((window_size, 1), dtype=float)
        # normalized time from [-1.0*(1-pct_delay) pct_delay]
        pct_delay = max(min(pct_delay, 1.0), 0.0)
        Xarray = np.array([np.linspace(-1.0*(1.0-pct_delay), pct_delay, window_size)], dtype=float).T
        # generate D array of Chebychev polynomials
        D = np.concatenate((Xarray**0.0, Xarray**1.0), 1)
        for i in range(2, filter_order+1):
            D = np.concatenate((D, (2*Xarray*np.array([D[:, i-1]]).T - np.array([D[:, i-2]]).T)), 1)
        sigma = np.eye(window_size, dtype=float)
        # generate the "gain" array (this math looks awful)
        self.K = np.dot(np.dot(la.inv(np.dot(np.dot(D.T, la.inv(sigma)), D)), D.T), sigma)
        # Save to memory some vectors. They represent the derivatives of the Chebychev polynomials evaluated at zero.
        self.f = np.array([1, 0, -1, 0, 1, 0, -1, 0, 1, 0, -1, 0], dtype=float)
        self.fp = np.array([0, 1, 0, -3, 0, 5, 0, -7, 0, 9, 0, -11], dtype=float)
        self.fpp = np.array([0, 0, 4, 0, -16, 0, 36, 0, -64, 0, 100, 0], dtype=float)
        self.fppp = np.array([0, 0, 0, 24, 0, -120, 0, 336, 0, -720, 0, 1320], dtype=float)
        # save the time at which measuremtns are expected to be provided..
        self.dt_frame = np.zeros(self.frame.shape, dtype=float) # this could be np.ones to avoid initialization issues

    def update(self, yk, dt):
        """Method is used to update the filter frame.

        Args:
            yk: current measurement as a float
            dt: time since the last measurement (seconds)

        Returns:
            No returns, only internal property changes
        """
        # shift current values in frame to the left
        self.frame = np.roll(self.frame, -1)
        # add the new measurement to the end
        self.frame[-1] = yk
        # in the same manner save the frame dt
        self.dt_frame = np.roll(self.dt_frame, -1)
        self.dt_frame[-1] = dt

    def return_state(self):
        """Method called to return a state and its derivatives from the filter
        window.

        Args:
            No arguments

        Returns:
            A list containing filtered [x, xd, xdd, xddd]
        """
        # determine the smoothed measurements
        Abar = np.dot(self.K, self.frame)
        # compute the mean dt in this frame
        dt = self.dt_frame.mean()
        # if dt is valid:
        if dt > 0.0:
            # compute values
            x = np.dot(self.f[0:(self.filter_order+1)], Abar)
            if self.filter_order > 0:
                xd = (np.dot(self.fp[0:(self.filter_order+1)], Abar) /
                         (dt*self.window_size))
            else:
                xd = np.array([0.0])
            if self.filter_order > 1:
                xdd = (np.dot(self.fpp[0:(self.filter_order+1)], Abar) /
                          (dt*self.window_size)**2.0)
            else:
                xdd = np.array([0.0])
            if self.filter_order > 2:
                xddd = (np.dot(self.fppp[0:(self.filter_order+1)], Abar) /
                           (dt*self.window_size)**3.0)
            else:
                xddd = np.array([0.0])
            # return
            return [(x.squeeze()).tolist(),
                    (xd.squeeze()).tolist(),
                    (xdd.squeeze()).tolist(),
                    (xddd.squeeze()).tolist()]
        else:
            return [0.0, 0.0, 0.0, 0.0]
