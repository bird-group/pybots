# -*- coding: utf-8 -*-
""" Filters based on dynamical systems -- ie, first and second order filters
"""

import numpy as np
import copy

class FirstOrderLowPass(object):
    """Simple class for doing low-pass RC filtering of a signal

    Attributes:
        filter_value: returns a filtered signal
        reset: resets the filter state
    """
    def __init__( self, dt, f_cutoff, u_init = 0.0 ):
        """ Constructor

        Arguments:
            dt: filter sample time in seconds
            f_cutoff: rolloff frequency in Hz
            u_init: initial value

        Returns:
            object
        """
        self.uk_1 = u_init
        rc = 1.0/(2.0*np.pi*f_cutoff)
        self.alpha = dt / (rc + dt)
        self.u_init = u_init

    def filter_value( self, value ):
        # check value first
        if (np.isfinite(value)):
            # compute current filtered value
            uk = self.uk_1 + self.alpha*( value - self.uk_1 )
            # save current value as previuos
            self.uk_1 = uk
            # return current filtered value
            return uk
        else:
            return self.uk_1

    def reset( self ):
        self.uk_1 = self.u_init

class RateLimitedLowPass(object):
    """An RC low-pass filter for smoothing the command output that also
    includes a rate limitng funciton."""
    def __init__(self, dt, f_cutoff, r_cutoff=1.0, u_init=0.0, is_angle=False):
        """ Constructor

        Arguments:
            dt: filter sample time in seconds
            f_cutoff: rolloff frequency in Hz
            r_cutoff: maximum rate of the signal in (signal/s)
            u_init: initial value
            is_angle: bool, defaults false, if true then wrap the signal between
                -pi and pi

        Returns:
            object
        """

        # dt in seconds, f_cutoff in Hz, r_cutoff in units/sec
        self.uk_1 = u_init
        self._is_angle = is_angle
        rc = 1.0/(2.0*np.pi*f_cutoff)
        self.alpha = dt / (rc + dt)
        self.u_init = u_init
        self.r_lim = r_cutoff*dt

    def filter_value(self, value):
        """Method takes the current measurement, filters it, and returns a
        filtered version.
        """
        # check value first
        if np.isfinite(value):
            if not self._is_angle:
                # compute current filtered value
                uk = self.uk_1 + self.alpha*(value - self.uk_1)
                # now rate-limit
                uk = self._rate_limit(uk)
            else:
                # in the case that we're filtering an angle, first wrap the error
                uk = self.uk_1 + self.alpha*(((value - self.uk_1) + np.pi)%(2.0 * np.pi) - np.pi)
                # now rate-limit
                uk = self._rate_limit(uk)
                # then wrap the output
                uk = (uk + np.pi)%(2.0 * np.pi) - np.pi
            # save current value as previuos
            self.uk_1 = uk
            # return current filtered value
            return uk
        else:
            return self.uk_1

    def _rate_limit(self, uk):
        """Method rate limits the input value."""
        if (uk - self.uk_1) > self.r_lim:
            return self.uk_1 + self.r_lim
        elif (uk - self.uk_1) < -1.0*self.r_lim:
            return self.uk_1 - self.r_lim
        else:
            return uk

    def reset(self, value=None):
        """Method is used to reset the LowPass Filter."""
        if value == None:
            self.uk_1 = self.u_init
        else:
            self.uk_1 = value

class SecondOrderSystem(object):
    """ A filter based on a second order system
    """
    def __init__( self ):
        """ Constructor, for the moment the values come from matlab, so this
        isn't a super-general-use filter yet
        """
        # unfortunately this filter is tuned in Matlab for now, so these values are hard-coded
#        tr = 3.0               % rie time in seconds
#        dt = 0.1
#        z = 0.707              % critically damped system
#        wn = 2.0/(z*tr)        % rise time approximation for critically damped system
#        a = [1,2*wn*z,wn^2];
#        b = [0,0,wn^2];
#        sys = tf(b,a)
#        sys_d = c2d(sys,dt)
#        sys_ss = ss(sys_d)
#        A = sys_ss.a
#        B=sys_ss.b
#        C = sys_ss.c
        self.A = np.array([[1.8669,-0.8752],[1.0,0]])
        self.B = np.array([[0.1250],[0.0]])
        self.C = np.array([0.0340, 0.0325])
        self.x = np.zeros([2,1])

    def filter_value( self, uk ):
        #check input value
        if (np.isfinite(uk)):
            self.x = np.dot( self.A, self.x ) + np.dot( self.B, uk )
        # return Y as a float
        return float(np.dot( self.C, self.x ))

    def reset( self ):
        self.x = np.zeros([2,1])

class DeltaDetect(object):
    """ A filter to do all the work of detecting a change in something
    """
    def __init__(self, x0=0):
        """ Constructor

        Arguments:
            x0: initial filter state

        Returns:
            the object
        """
        super(DeltaDetect, self).__init__()
        self.x = copy.deepcopy(x0)

    def is_x_changed(self, x):
        """ Check whether x has changed, update x

        Arguments:
            x: new value

        Returns:
            bool, true for changed
        """
        x_last = copy.deepcopy(self.x)
        self.x = copy.deepcopy(x)
        if self.x == x_last:
            return False
        else:
            return True
