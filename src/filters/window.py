import copy
import pdb

import numpy

import geodesy.conversions

class Average(object):
    def __init__(self, n=None, n_max=numpy.inf, data=[], weights=[], dfun=None):
        """Constructor

        Arguments:
            n: the number of samples to average over. Optional but must be
                specified when taking the average if not set
            n_max: the maximum number of points to keep. Defaults to inf so this
                will be a memory leak if this is not set!!!
            data: initial data, optional
            weights: initial weights, optional
            dfun: optional function to apply to data before averaging. default
                simply averages data

        Returns:
            class instance
        """
        self._n = n
        self._n_max = n_max
        self._data = copy.deepcopy(data)
        self._weights = copy.deepcopy(weights)
        if dfun is None:
            dfun = lambda x: x
        self._dfun = dfun

    def add(self, x, w=1.0):
        """add a new value

        Arguments:
            x: new value (can be list or numpy array of values)
            w: weight. optional, if specified must match dimension and type of x

        Returns:
            no returns
        """
        self._data.append(x)
        self._weights.append(w)
        if len(self._data) > self._n_max:
            self._data.pop(0)
        if len(self._weights) > self._n_max:
            self._weights.pop(0)

    def set_n(self, n):
        """Set the window length

        Arguments:
            n: number of samples to include in window

        Returns:
            no returns
        """
        self._n = n

    def set_n_max(self, n_max):
        """Set the number of samples saved

        Arguments:
            n_max: number of samples to save

        Returns:
            no returns
        """
        self._n_max = n_max

    def mean(self, n=None):
        """Get the value of this window filter

        Arguments:
            n: use this value for the filter length. optional, can use internal
                value for n

        Returns:
            v: value of the filter data over the previous n samples
        """
        if n is None:
            assert self._n is not None, 'n must be set if not specified'
            n = self._n

        data = self._dfun(numpy.array(self._data[-n:]))
        weights = numpy.array(self._weights[-n:])
        weights /= numpy.sum(weights)

        return numpy.sum(data.T * weights, axis=-1).T

    def var(self, n=None):
        """Get the variance of this window filter

        Arguments:
            n: use this value for the filter length. optional, can use internal
                value for n

        Returns:
            var: value of the filter data over the previous n samples
        """
        if n is None:
            assert self._n is not None, 'n must be set if not specified'
            n = self._n

        mean = self.mean(n)
        data = self._dfun(numpy.array(self._data[-n:]))
        weights = numpy.array(self._weights[-n:])
        weights /= numpy.sum(weights)
        return (
            numpy.sum(numpy.power(data - mean, 2.0).T * weights, axis=-1).T /
            (1.0 - numpy.sum(numpy.power(weights, 2.0)))
            )

    def window(self, n=None):
        if n is None:
            assert self._n is not None, 'n must be set if not specified'
            n = self._n

        return self._dfun(numpy.array(self._data[-n:]))

class TimeAverage(object):
    def __init__(
        self, t=None, t_max=numpy.inf, data=[], t0=[], weights=[], dfun=None):
        """Constructor

        Arguments:
            t: the length of time average over. Optional but must be
                specified when taking the average if not set
            t_max: the time to keep. Defaults to inf so this
                will be a memory leak if this is not set!!!
            data: initial data, optional
            t0: initial time, optional
            weights: initial weights, optional
            dfun: optional function to apply to data before averaging. default
                simply averages data

        Returns:
            class instance
        """
        self._t_avg = t
        self._t_max = t_max
        self._data = copy.deepcopy(data)
        self._t = copy.deepcopy(t0)
        self._weights = copy.deepcopy(weights)

        if dfun is None:
            dfun = lambda x: x
        self._dfun = dfun

    def add(self, x, t, w=1.0):
        """add a new value

        Arguments:
            x: new value (can be number, list, or numpy array of values)
            t: new data (can be number, list, or numpy array of value)
            w: weight. optional, if specified must match dimension and type of x

        Returns:
            no returns
        """
        self._data.append(x)
        self._t.append(t)
        self._weights.append(w)
        while (self._t[-1] - self._t[0]) > self._t_max:
            self._data.pop(0)
            self._t.pop(0)
            self._weights.pop(0)

    def set_t(self, t):
        """Set the window length

        Arguments:
            t: window length in seconds

        Returns:
            no returns
        """
        self._t_avg = t

    def set_t_max(self, t_max):
        """Set the length of time to save

        Arguments:
            t_max: max time to save (seconds)

        Returns:
            no returns
        """
        self._t_max = t_max
        while (self._t[-1] - self._t[0]) > self._t_max:
            self._data.pop(0)
            self._t.pop(0)
            self._weights.pop(0)

    def mean(self, t_avg=None, t_ref=None):
        """Get the value of this window filter

        Arguments:
            t_avg: use this value for the window length. optional, can use internal
                value for t
            t_ref: use this for the reference time when computing the age of
                each sample. Optional, defaults to the last sample time

        Returns:
            v: value of the filter data over the previous n samples
        """
        tslice = self.slice(t_avg, t_ref)

        weights = numpy.array(self._weights)[tslice]
        weights /= numpy.sum(weights)

        mean = numpy.sum(
            self._dfun(numpy.array(self._data)[tslice]).T * weights,
            axis=-1).T
        return mean

    def var(self, t=None, t_ref=None):
        """Get the variance of this window filter

        Arguments:
            t: use this value for the window length. optional, can use internal
                value for t
            t_ref: use this for the reference time when computing the age of
                each sample. Optional, defaults to the last sample time

        Returns:
            var: value of the filter data over the previous n samples
        """
        tslice = self.slice(t_avg, t_ref)

        weights = numpy.array(self._weights)[tslice]
        weights /= numpy.sum(weights)
        data = self.dfun(numpy.array(self._data)[tslice])

        mean = self.mean(t, t_ref)
        return (
            numpy.sum(numpy.power(data - mean, 2.0) * weights, axis=-1) /
            (1.0 - numpy.sum(numpy.power(weights, 2.0)))
            )

    def slice(self, t_avg=None, t_ref=None):
        """Get the slice of data to work with

        Arguments:
            t: use this value for the window length. optional, can use internal
                value for t
            t_ref: use this for the reference time when computing the age of
                each sample. Optional, defaults to the last sample time

        Returns:
            tslice: array slice index for saved data
        """
        if t_avg is None:
            assert self._t is not None, 't must be set if not specified'
            t_avg = self._t_avg

        if t_ref is None:
            t_ref = self._t[-1]
        dt = t_ref - numpy.array(self._t)
        tslice = dt < t_avg
        return tslice

    def window(self, t_avg=None, t_ref=None):
        """Get the window of data

        Arguments:
            t: use this value for the window length. optional, can use internal
                value for t
            t_ref: use this for the reference time when computing the age of
                each sample. Optional, defaults to the last sample time

        Returns:
            tslice: array slice index for saved data
        """
        tslice = self.slice(t_avg, t_ref)
        return self._dfun(numpy.array(self._data)[tslice]).T

class GeodesicAverage(Average):
    """A queue which has lat/long/alt as the first three elements of its data

    The LLA points will be converted to NED relative to the last point before
    averaging and then converted back, avoiding nonlinearity issues when
    averaging LLA
    """
    def __init__(self, n=None, n_max=numpy.inf, data=[], weights=[]):
        """Constructor

        Arguments:
            n: the number of samples to average over. Optional but must be
                specified when taking the average if not set
            n_max: the maximum number of points to keep. Defaults to inf so this
                will be a memory leak if this is not set!!!
            data: initial data, optional
            weights: initial weights, optional

        Returns:
            class instance
        """
        super(GeodesicAverage, self).__init__(
            n, n_max, data, weights, self._lla_to_ned)

    def _lla_to_ned(self, x):
        """Convert the first three elements of the data to ned for averaging
        """
        x[:,:3] = geodesy.conversions.lla_to_ned(
            x[:,:3], numpy.array(self._data[-1][:3], ndmin=2))
        return x

    def _ned_to_lla(self, x):
        x = numpy.array(x, ndmin=2)
        x[:,:3] = geodesy.conversions.ned_to_lla(
            x[:,:3], numpy.array(self._data[-1][:3], ndmin=2))
        return numpy.squeeze(x)

    def mean(self, n=None):
        """Get the value of this window filter

        Arguments:
            n: use this value for the filter length. optional, can use internal
                value for n

        Returns:
            v: value of the filter data over the previous n samples
        """
        mean = super(GeodesicAverage, self).mean(n)
        return self._ned_to_lla(mean)

class GeodesicTimeAverage(TimeAverage):
    """A queue which has lat/long/alt as the first three elements of its data

    The LLA points will be converted to NED relative to the last point before
    averaging and then converted back, avoiding nonlinearity issues when
    averaging LLA
    """
    def __init__(
        self, t=None, t_max=numpy.inf, data=[], t0=[], weights=[]):
        """Constructor

        Arguments:
            t: the length of time average over. Optional but must be
                specified when taking the average if not set
            t_max: the time to keep. Defaults to inf so this
                will be a memory leak if this is not set!!!
            data: initial data, optional
            t0: initial time, optional
            weights: initial weights, optional

        Returns:
            class instance
        """
        super(GeodesicTimeAverage, self).__init__(
            t, t_max, data, t0, weights, self._lla_to_ned)

    def _lla_to_ned(self, x):
        """Convert the first three elements of the data to ned for averaging
        """
        x[:,:3] = geodesy.conversions.lla_to_ned(
            x[:,:3], numpy.array(self._data[-1][:3], ndmin=2))
        return x

    def _ned_to_lla(self, x):
        x = numpy.array(x, ndmin=2)
        x[:,:3] = geodesy.conversions.ned_to_lla(
            x[:,:3], numpy.array(self._data[-1][:3], ndmin=2))
        return numpy.squeeze(x)

    def mean(self, t_avg=None, t_ref=None):
        """Get the value of this window filter

        Arguments:
            t: use this value for the window length. optional, can use internal
                value for t
            t_ref: use this for the reference time when computing the age of
                each sample. Optional, defaults to the last sample time

        Returns:
            v: value of the filter data over the previous n samples
        """
        mean = super(GeodesicTimeAverage, self).mean(t_avg, t_ref)
        return self._ned_to_lla(mean)

