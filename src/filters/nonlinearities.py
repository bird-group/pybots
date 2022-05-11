"""Implement some nonlinearies
"""
import numpy

import scipy.interpolate

class DelayLine(object):
    """Delay a signal
    """
    def __init__(self, delay=None):
        """Constructor

        Arguments:
            delay: time to delay the signal by, defaults to no delay (s)

        Returns:
            class instance
        """
        self._filter_state = None
        self._delay = delay

        self._signal_queue = []
        self._time_queue = []

    def filter_value(self, value, time):
        """Filter a new value

        Returns None if there are not any samples old enough. If the timestamp
        is less than any time currently in the queue, the filter will reset

        Arguments:
            value: the new value to filter
            time: the stamp for that time

        Returns:
            filtered: the new state of the filter
        """
        if self._delay is None:
            self._filter_state = value
            return value

        self._signal_queue.append(value)
        self._time_queue.append(time)

        if time < numpy.amax(self._time_queue):
            self.reset()
            return None

        filtered = None
        while (
            (time - self._time_queue[0]) > self._delay and
            len(self._time_queue) > 0):
            filtered = self._signal_queue.pop(0)
            self._time_queue.pop(0)

        self._filter_state = filtered
        return filtered

    @property
    def value(self):
        """Return the current value of the filter

        Assumes time starts with the first element in the filter. Returns None
        if there are not any samples old enough

        Arguments:
            no arguments

        Returns:
            value: current filter value
        """
        return self._filter_state

    def value_at_time(self, time):
        """Return the value of the filter for a specified time

        Returns None if there are not any samples old enough

        Arguments:
            time: the time to query

        Returns:
            value: filter value
        """
        filtered = None
        idx = len(self._time_queue) - 1
        while (
            (time - self._time_queue[idx]) > self._delay and
            idx > 0):
            filtered = self._signal_queue[idx]
            idx -= 1

        return filtered

    def set_delay(self, delay):
        """Set the delay time value

        Arguments:
            delay: the time to delay the signal for

        Returns:
            delay: delay duration (s)
        """
        self._delay = delay

    def reset(self):
        """Reset the delay queue

        Arguments:
            no arguments

        Returns:
            no returns
        """
        self._time_queue = []
        self._signal_queue = []

class Deadband(object):
    """Implement a deadband
    """
    def __init__(self, center, width=None, width_fun=None):
        """Constructor

        Arguments:
            center: center of the deadband
            width: width, can be a scalar for symmetric deadband or a vector
                for variable upper / lower bounds. If center is a N vector then
                this should be Nx2 matrix of bounds. Optional (can specify fun)
            width_fun: width function, evaluates a point and returns True if
                the point is within the deadband. Optional (can specify width
                instead)

        Returns:
            no returns
        """
        self._center = center

        assert width is not None or width_fun is not None,\
            'Must specify width'

        if isinstance(center, numpy.ndarray) and width is not None:
            assert center.shape[0] == width.shape[0],\
                'for N element center, width must be Nx2'
            assert width.ndim == 2,\
                'for N element center, width must be Nx2'
            assert width.shape[1] == 2,\
                'for N element center, width must be Nx2'

        if width is not None:
            self._width_vector = width
            self._width = self._square_fun
        else:
            self._width = width_fun

    def _square_fun(self, point):
        """Evaluate a point to see if it falls within a bracketed range

        This evalutes a square ball around a point and returns true if the point
        lies within it

        Arguments:
            point: location to be tested

        Returns:
            in_interior: true if the point lies within the interior of the ball
        """
        point -= self._center

        if isinstance(self._center, numpy.ndarray):
            for x, bound in zip(self._width_vector, point):
                if x < numpy.amin(bound) or x > numpy.amax(bound):
                    return False
            return True

        if (
            point < numpy.amin(self._width_vector) or
            point > numpy.amax(self._width_vector)):
            return False

        return True

    def filter_value(self, value):
        """filter a signal through the deadband

        Arguments:
            value: the value we want to filter

        Returns:
            filtered: that value filtered through a deadband
        """
        if self._width(value):
            return self._center

        return value
