import numpy

class ClippingFilter(object):
    """Clips a value to given limits
    """
    def __init__(self, min_value, max_value, u_init=None):
        """Constructor

        Arguments:
            min_value: the minimum value permitted for the output
            max_value: the maximum value permitted for the output
            u_init: optional, initial value, must lie in [min_value, max_value]
                if not specified will be set to mean(min_value, max_value)

        Returns:
            class instance
        """
        if u_init is not None:
            assert u_init >= min_value and u_init <= max_value,\
                'u_init must lie in [min_value, max_value]'
            self._u = u_init
        else:
            self._u = (min_value + max_value) / 2.0
        self._min_value = min_value
        self._max_value = max_value

    def filter_value(self, value):
        """Filter a new realization of u and return the filtered value

        Arguments:
            value: a new value for u which should be filtered

        Returns:
            u_filt: filtered value
        """
        self._u = numpy.clip(value, self._min_value, self._max_value)
        return self._u

    @property
    def u(self):
        """Getter for current value of the filter

        Returns:
            u: current filter value
        """
        return self._u

class RateLimiter(object):
    """A simple rate limiter
    """
    def __init__(self, max_slew, dt=1.0):
        """Constructor

        Arguments:
            max_slew: the maximimum allowable slew rate of the signal
            dt: the timestep size of this filter. Optional, defaults to 1. A
                timestep can also be specified at the filter time

        Returns:
            class instance
        """
        self._x = None
        self._t_1 = None

        self._dt = dt
        self._max_slew = max_slew

    def filter_value(self, new_value, t=None):
        """Filter a value

        Arguments:
            new_value: new candidate value to rate limit
            t: optional, the epoch this value occurs at. The timestep size
                will be computed with this and the last time specified. If not
                specified then the timestep specified at construction will be
                used (which defaults to 1.0)

        Returns:
            filter_output: the rate limited output
        """
        if self._x is None:
            self._x = new_value
            return self._x

        if t is not None and self._t_1 is None:
            self._t_1 = t
            return self._x

        if t is not None:
            dt = t - self._t_1
            self._t_1 = t
        else:
            dt = self._dt

        slew_rate = (new_value - self._x) / dt
        if numpy.abs(slew_rate) > self._max_slew:
            self._x += self._max_slew * numpy.sign(slew_rate) * dt
        else:
            self._x = new_value
        return self._x


