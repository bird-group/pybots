# -*- coding: utf-8 -*-
"""
Created on Fri Mar 20 12:37:05 2015

@author: Nate
"""
import pdb
import warnings

import copy

import numpy
import scipy.signal

class LowPassRCFilter(object):
    """Simple class for doing low-pass RC filtering of a signal

    Attributes:
        filter_value: returns a filtered signal
        reset: resets the filter state
    """
    def __init__(self, dt, f_cutoff, u_init=0.0, is_angle=False):
        # dt in seconds, f_cutoff in Hz
        self.uk_1 = u_init
        self._is_angle = is_angle
        self.u_init = u_init
        self.set_parameters(dt, f_cutoff)

    def set_parameters(self, dt, f_cutoff):
        rc = 1.0/(2.0*numpy.pi*f_cutoff)
        self._alpha = dt / (rc + dt)

    def filter_value(self, value):
        # check value first
        if not numpy.all(numpy.isfinite(value)):
            return self.uk_1

        innovation = value - self.uk_1
        if self._is_angle:
            # if we're filtering an angle, wrap the error
            innovation = geometry.rotations.angle_difference(value, self.uk_1)

        self.uk_1 += self._alpha * innovation
        # return current filtered value
        return self.uk_1

    def reset(self, value=None):
        """Method is used to reset the LowPass Filter."""
        if value == None:
            self.uk_1 = self.u_init
        else:
            self.uk_1 = value

    @property
    def value(self):
        """Get the current filter value

        Arguments:
            no arguments

        Returns:
            value: the current filter value (which would have also been
                returned at the latest call of filter_value)
        """
        return self.uk_1

class HighPassRCFilter(object):
    """Simple class for doing high-pass RC filtering of a signal
    """
    def __init__(self, dt, f_cutoff, u_init=0.0):
        # dt in seconds, f_cutoff in Hz
        self.uk_1 = u_init
        self.u_init = u_init
        self.set_parameters(dt, f_cutoff)
        self._innovation = 0.0

    def set_parameters(self, dt, f_cutoff):
        rc = 1.0 / (2.0 * numpy.pi * f_cutoff)
        self._alpha = rc / (rc + dt)

    def filter_value(self, value):
        # check value first
        if not numpy.all(numpy.isfinite(value)):
            return self.uk_1

        self.uk_1 = self._alpha * (value + self._innovation)
        self._innovation = self.uk_1 - value
        # return current filtered value
        return self.uk_1

    def reset(self, value=None):
        """Method is used to reset the LowPass Filter."""
        self._innovation = 0.0
        if value == None:
            self.uk_1 = self.u_init
        else:
            self.uk_1 = value

    @property
    def value(self):
        """Get the current filter value

        Arguments:
            no arguments

        Returns:
            value: the current filter value (which would have also been
                returned at the latest call of filter_value)
        """
        return self.uk_1

class VariableTimestepLowpassFilter(object):
    """Class for doing a simple RC filter on a signal with variable sample time

    Note that there are some basic assumptions you shouldn't violate...like make
    sure that the timestep size and cutoff frequency are nyquist compatible
    """
    def __init__(self, f_cutoff, u_init='clear', is_angle=False):
        """Constructor

        Arguments:
            f_cutoff: cutoff frequency (Hz)
            u_init: optional, initial filter state, if unspecified the first
                value filtered will be used
            is_angle: optional, boolean defaults false, indicating if the signal
                being filtered is an angle (so it is restricted to [-pi, pi]

        Returns:
            class instance
        """
        self._uk_1 = u_init
        self._is_angle = is_angle

        # the choice of 'clear' as a default is unconventional, but when
        # resetting the filter, None was already claimed as the way to force
        # a reset to the initial filter value. (this choice was made in
        # LowPassRCFilter and ubiquitous in our code) So an alternate means was
        # required in order to indicate a condition where the filter should
        # initialize itself to the first specified value
        self._u_init = u_init
        self._f_cutoff = f_cutoff

    def set_cutoff_frequency(self, f_cutoff):
        """Set the cutoff frequency

        Arguments:
            f_cutoff: cutoff frequency (Hz)

        Returns:
            no returns
        """
        self._f_cutoff = f_cutoff

    def filter_value(self, new_sample, dt):
        """Incorporate a new sample into the filter

        Arguments:
            new_sample: the new value to apply the filter to
            dt: the time since the previous sample

        Returns:
            x: the new filter value
        """
        if self._uk_1 == 'clear':
            self._uk_1 = new_sample
            return self._uk_1

        # keep bad numbers out of IIR filters to avoid numerical drama
        if not numpy.all(numpy.isfinite(new_sample)):
            return self._uk_1

        rc = 1.0 / (2.0 * numpy.pi * self._f_cutoff)
        alpha = dt / (rc + dt)

        innovation = new_sample - self._uk_1
        if self._is_angle:
            # in the case that we're filtering an angle, wrap the error
            innovation = geometry.rotations.angle_difference(
                new_sample, self._uk_1)

        self._uk_1 += alpha * innovation
        # return current filtered value
        return self._uk_1

    def reset(self, value=None, clear=False):
        """Method is used to reset the LowPass Filter.

        Arguments:
            value: optional, if not specified the filter will take on the
                initial value specified at construction
            clear: optional boolean. Defaults False, if set True then the filter
                will take the next value given to it
        """
        if clear:
            self._uk_1 = 'clear'
            return

        if value is None:
            self._uk_1 = self.u_init
        else:
            self._uk_1 = value

    @property
    def value(self):
        """Get the current filter value

        Arguments:
            no arguments

        Returns:
            value: the current filter value (which would have also been
                returned at the latest call of filter_value)
        """
        return self._uk_1

class VariableTimestepHighpassFilter(object):
    """Class for doing a simple RC highpass filter on a signal with variable
        sample time

    Note that there are some basic assumptions you shouldn't violate...like make
    sure that the timestep size and cutoff frequency are nyquist compatible
    """
    def __init__(self, f_cutoff, u_init='clear'):
        """Constructor

        Arguments:
            f_cutoff: cutoff frequency (Hz)
            u_init: optional, initial filter state, if unspecified the first
                value filtered will be used

        Returns:
            class instance
        """
        self._uk_1 = u_init

        # the choice of 'clear' as a default is unconventional, but when
        # resetting the filter, None was already claimed as the way to force
        # a reset to the initial filter value. (this choice was made in
        # LowPassRCFilter and ubiquitous in our code) So an alternate means was
        # required in order to indicate a condition where the filter should
        # initialize itself to the first specified value
        self._u_init = u_init
        self._f_cutoff = f_cutoff
        self._innovation = 0.0

    def set_cutoff_frequency(self, f_cutoff):
        """Set the cutoff frequency

        Arguments:
            f_cutoff: cutoff frequency (Hz)

        Returns:
            no returns
        """
        self._f_cutoff = f_cutoff

    def filter_value(self, new_sample, dt):
        """Incorporate a new sample into the filter

        Arguments:
            new_sample: the new value to apply the filter to
            dt: the time since the previous sample

        Returns:
            x: the new filter value
        """
        if self._uk_1 == 'clear':
            self._uk_1 = new_sample
            return self._uk_1

        # keep bad numbers out of IIR filters to avoid numerical drama
        if not numpy.all(numpy.isfinite(new_sample)):
            return self._uk_1

        rc = 1.0 / (2.0 * numpy.pi * self._f_cutoff)
        alpha = rc / (rc + dt)

        self._uk_1 = alpha * (new_sample + self._innovation)
        self._innovation = (self._uk_1 - new_sample)
        # return current filtered value
        return self._uk_1

    def reset(self, value=None, clear=False):
        """Method is used to reset the LowPass Filter.

        Arguments:
            value: optional, if not specified the filter will take on the
                initial value specified at construction
            clear: optional boolean. Defaults False, if set True then the filter
                will take the next value given to it
        """
        self._innovation = 0.0
        if clear:
            self._uk_1 = 'clear'
            return

        if value is None:
            self._uk_1 = self.u_init
        else:
            self._uk_1 = value

    @property
    def value(self):
        """Get the current filter value

        Arguments:
            no arguments

        Returns:
            value: the current filter value (which would have also been
                returned at the latest call of filter_value)
        """
        return self._uk_1

class VariableTimestepRCFilter(VariableTimestepLowpassFilter):
    """For backward compatibility with the previous name of this class"""
    def __init__(self, f_cutoff, u_init='clear', is_angle=False):
        super(VariableTimestepRCFilter, self).__init__(
            f_cutoff, u_init, is_angle)
        warnings.warn(
            "Deprecated, use VariableTimestepLowpassFilter instead",
            DeprecationWarning)

class RateLimitedLowPass(object):
    """An RC low-pass filter for smoothing the command output."""
    def __init__(self, dt, f_cutoff, r_cutoff=1.0, u_init=0.0, is_angle=False):
        # dt in seconds, f_cutoff in Hz, r_cutoff in units/sec
        self.uk_1 = u_init
        self._is_angle = is_angle
        self.u_init = u_init
        self.set_parameters(dt, f_cutoff, r_cutoff)

    def set_parameters(self, dt, f_cutoff, r_cutoff=1.0):
        rc = 1.0/(2.0*numpy.pi*f_cutoff)
        self._alpha = dt / (rc + dt)
        self.r_lim = r_cutoff*dt

    def filter_value(self, value):
        """Method takes the current measurement, filters it, and returns a
        filtered version.
        """
        # check value first
        if not numpy.all(numpy.isfinite(value)):
            return self.uk_1

        innovation = value - self.uk_1
        if self._is_angle:
            innovation = geometry.rotations.angle_difference(value, self.uk_1)

        uk = self.uk_1 + self._alpha * innovation
        uk = self._rate_limit(uk)

        self.uk_1 = uk
        # return current filtered value
        return uk

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

class Debounce(object):
    """Class for debouncing booleans. Turns a boolean into a continuous
    variable with first-order dynamics."""
    def __init__(self, dt, rise_time, start_cond=False):
        """Initializes Debounce class.

        Args:
            dt: expected interval between updates (sec)
            trise: rise time for a low-pass RC filter, 20% to 80%
            start condition of the button, (bool)
        """
        if start_cond:
            self.filter_state = 1.0
        else:
            self.filter_state = 0.0
        # save the initial state of the filter
        self._initial_state = self.filter_state
        # set internal filter parameters
        self.set_parameters(dt, rise_time)

    def set_parameters(self, dt, rise_time):
        # RC filter alpha parameter (weight on new measurement):
        # http://en.wikipedia.org/wiki/RC_time_constant
        self._alpha = dt / ((rise_time/(2.0*numpy.pi*0.22)) + dt)

    def press(self, data):
        """Main method of the class, call whenever there is new information to
        update the internal state.

        Args:
            data: boolean to update internal state
        """
        if data:
            self.filter_state += self._alpha * (1.0 - self.filter_state)
        else:
            self.filter_state += self._alpha * (0.0 - self.filter_state)

    def reset(self, data=None):
        """Reset method for the class.

        Args:
            data: if specified, indicates the reset value for the boolean.
        """
        if data == None:
            self.filter_state = self._initial_state
        else:
            if data:
                self.filter_state = 1.0
            else:
                self.filter_state = 0.0

    def is_true(self):
        """Method returns true if the internal state is trueish, otherwise
        returns false. This is the less confusing method to use."""
        if self.filter_state > 0.75: # may modify this value..
            return True
        return False

    def is_false(self):
        """Method returns true if the internal state is falseish, otherwise
        returns false."""
        if self.filter_state < 0.25:
            return True
        return False

class Sigmoid(object):
    """ Class for a logistic sigmoid function used for debouncing boolean
    decisions."""
    def __init__(self, dt, rise_time, start_cond=False):
        """Initializes logistic sigmoid class.

        Args:
            dt: expected interval between updates (sec)
            rise_time: rise time for a the sigmoid, full false to above
                threshold
            start condition of the button, (bool)
        """
        # define max and min limits for log-odds
        self._odds_min = -6.0
        self._odds_max = 6.0
        # set the start condition of the sigmoid
        if start_cond:
            # start true
            self.log_odds = self._odds_max
        else:
            # start false
            self.log_odds = self._odds_min
        # save the initial state of the filter
        self._initial_odds = self.log_odds
        # define the threshold for this sigmoid to return true
        self.threshold = 0.9
        # variable defines the rise-time of the sigmoid
        # (full negative to over threshold)
        self.set_parameters(dt, rise_time)

    def set_parameters(self, dt, rise_time):
        """Method used to set the slope of the sigmoid.

        Args:
            dt: expected call interval for the update (seconds)
            rise_time: desired rise time for the sigmoid (full false to above
                threshold true)
        """
        # RC filter alpha parameter (weight on new measurement):
        # http://en.wikipedia.org/wiki/RC_time_constant
        self._alpha = (
            (self._odds_max - numpy.log(1.0 / self.threshold - 1.0))
            * dt /
            rise_time)

    def press(self, data):
        """Main method of the class, call whenever there is new information to
        update the internal state.

        Args:
            data: boolean to update internal state
        """
        if data:
            return self._update(1.0)
        else:
            return self._update(-1.0)

    def _update(self, Xi, wi=1.0):
        """Method updates the log-odds before returning the current
        probability.

        Args:
            Xi: current observation (ideally [-1, 1])
            wi: weight on the current observaton (default 1.0)

        Returns:
            float probability that the state is high
        """
        self.log_odds += self._alpha * (Xi * wi)
        self.log_odds = numpy.clip(
            self.log_odds, self._odds_min, self._odds_max)
        return self.probability

    def reset(self, data=None):
        """Reset method for the class.

        Args:
            data: if specified, indicates the reset value for the boolean.
        """
        if data == None:
            self.log_odds = self._initial_odds
        else:
            if data:
                self.log_odds = self._odds_max
            else:
                self.log_odds = self._odds_min

    def is_true(self):
        """Method returns true if the internal state is trueish, otherwise
        returns false. This is the less confusing method to use."""
        if self.probability > self.threshold:
            return True
        return False

    def is_false(self):
        """Method returns true if the internal state is falseish, otherwise
        returns false."""
        if self.probability < (1.0 - self.threshold):
            return True
        return False

    @property
    def probability(self):
        """Recover the current probability from the sigmoid."""
        return 1.0 / (1.0 + numpy.exp(-1.0*(self.log_odds)))

class SecondOrderSystem(object):
    """Class implements a simple overdamped second order system for smoothing
    commands."""
    def __init__(self, dt, rise_time, z=0.707):
        """Initializes the SecondOrderSystem class.

        Args:
            dt: timestep at which filter_value is called (s)
            tr: rise time (s)
        """
        self.set_parameters(dt, rise_time, z)
        self.x = numpy.zeros([2, 1])

    def filter_value(self, uk):
        """Method advances the state of the filter and returns the current
        filtered value.

        Args:
            uk: measurement

        Returns:
            x: filtered value
        """
        #check input value
        if numpy.isfinite(uk):
            # update the filter state
            self.x = numpy.dot(self.A, self.x) + numpy.dot(self.B, uk)
        # return Y as a float
        return self.return_state()

    def set_parameters(self, dt, rise_time, z=0.707):
        """Sets the parameters of the second orger system.

        Args:
            dt: timestep at which filter_value is called (s)
            tr: rise time (s)
            z: damping ratio
        """
        # rise time approximation for critically damped system
        wn = 2.0/(z*rise_time)
        a = [1.0, 2.0*wn*z, wn**2]
        b = [0.0, 0.0, wn**2]
        sysd = scipy.signal.cont2discrete([b, a], dt)
        [self.A, self.B, self.C, _] = scipy.signal.tf2ss(sysd[0], sysd[1])

    def reset(self, value=None):
        """Note that reset to a non-zero value isn't implemented because it's a
        kinda hard problem. 1 input, 2 unknowns."""
        self.x = numpy.zeros([2, 1])

    def return_state(self):
        """Method returns the state of the second order system when called.
        Note that the response will be one dimensional, even though there are
        technically two internal states."""
        return float(numpy.dot(self.C, self.x))

class RecursiveAverager(object):
    """Implements a recursive averager
    """
    def __init__(self, x0=None):
        """Constructor

        Arguments:
            x0: optional, the initial filter state. If unspecified then the
                first filtered value will be used to initialize the filter

        Returns:
            class instance
        """
        self.reset(x0)

    def filter_value(self, new_sample):
        """Filter a new sample

        Arguments:
            new_sample: the new value we'd like to filter

        Returns:
            x: the updated filter value
        """
        self._x = (self._x * self._n + new_sample) / (self._n + 1)
        self._n += 1
        return self._x

    def reset(self, x0=None):
        """Reset the filter

        Arguments:
            x0: optional, the initial filter state. If unspecified then the
                first filtered value will be used to initialize the filter

        Returns:
            no returns
        """
        if x0 is None:
            self._x = 0.0
            self._n = 0
        else:
            self._x = x0
            self._n = 1

    @property
    def value(self):
        """Return the current filter value

        Arguments:
            no arguments

        Returns:
            value: the current filter value
        """
        return self._x

class IIRFilter(object):
    """A generic IIR filter implemented using [b,a] vectors

    This follows the MATLAB (and apparently scipy) filter syntax. From the
    MATLAB help for "filter" it is a "Direct Form II Transposed" form

    a[0] * y[n] = b[0] * x[n] + b[1] * x[n-1] + ... + b[nb+1] * x[n-nb] - ...
        a[1] * y[n-1] - ... - a[na+1] * y[n-na]
    """
    def __init__(self, b=None, a=None, u_init=0.0, iirfilter_args=None):
        """Constructor

        This assumes that the sample time is constant and known...

        Arguments:
            b: numpy array giving B vector
            a: numpy array giving A vector
            u_init: initial filter value
            iirfilter_args: optionally, if b and a are left unspecified then
                a dictionary which is a valid input to scipy.signal.iirfilter
                can be used to specify the filter

        Returns:
            class instance
        """
        if b is None and a is None:
            assert isinstance(iirfilter_args, dict),\
                'if a and b are not specified then iirfilter_args must be given'
            b, a = scipy.signal.iirfilter(**iirfilter_args)

        if isinstance(u_init, numpy.ndarray):
            self._n = u_init.shape[0]
        else:
            self._n = 1

        assert a.ndim == b.ndim, 'a and b must have the same dim'
        if a.ndim == 1:
            self._a = numpy.tile(a, (self._n, 1))
            self._b = numpy.tile(b, (self._n, 1))

        self._x = numpy.zeros((self._n, max(b.shape)))
        self._y = numpy.zeros((self._n, max(a.shape)))

        self._y[0] = u_init
        self._initial_value = u_init

    def reset(self, u_init=None, default=True):
        """Reset the filter

        Arguments:
            u_init: initialize the filter to this value, if not specified it
                will reset to the value used at construction
            default: optional, defaults True. use this value as the default
                reinitialization value going forward

        Returns:
            no returns
        """
        self._x = numpy.zeros((self._n, max(self._b.shape)))
        self._y = numpy.zeros((self._n, max(self._a.shape)))

        if u_init is None:
            self._y[0] = self._initial_value
            return

        self._y[0] = u_init

        if default:
            self._initial_value = u_init


    def filter_value(self, new_sample):
        """Filter a sample

        Arguments:
            new_sample: the new value to filter

        Returns:
            value: the latest filter output
        """
        self._x[:, 1:] = self._x[:, 0:-1]
        self._x[:, 0] = new_sample

        self._y[:, 1:] = self._y[:, 0:-1]
        self._y[:, 0] = 1.0 / self._a[:, 0] * (
            self._x.dot(self._b.T) - self._y[:, 1:].dot(self._a[:, 1:].T))

        return self.value

    @property
    def value(self):
        """Getter for the current filter value

        Arguments:
            no arguments

        Returns:
            value: the current filter value
        """
        return copy.deepcopy(numpy.squeeze(self._y[:,0]))
