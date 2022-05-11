import numpy

def ring_index(this_array, inds):
    """ Index an array as a ring

    Takes a numpy array and indexes it as if it were a ring, for instance,
    given a 10 element array, ring_index(arr, 0) == ring_index(arr, 10)

    Arguments:
        this_array: numpy array
        inds: index or list of indices

    Returns:
        new_array: indexes out the desired items
    """

    assert isinstance(this_array, numpy.ndarray), "Array must be numpy array"
    if numpy.isscalar(this_array):
        return this_array
    if type(inds) is int:
        inds = [inds]
    inds = [i % len(this_array) for i in inds]
    return this_array[inds]

def extrema(this_array, axis=None):
    """Get the extrema from a numpy array

    Arguments:
        this_array: numpy array to get the extrema from
        axis: axis to find the minima along

    Returns:
        min: minima of this array
        max: maxima of this array
    """
    extr = (
        numpy.amin(this_array, axis=axis), numpy.amax(this_array, axis=axis))
    return extr

def argextrema(this_array, axis=None):
    """Get the location of the extrema

    Arguments:
        this_array: numpy array to get the extrema location from
        axis: axis to search along

    Returns:
        argmin: argmin of this array
        argmax: argmax of this array
    """
    argextr = (
        numpy.argmin(this_array, axis=axis),
        numpy.argmax(this_array, axis=axis))
    return argextr

def nanextrema(this_array, axis=None):
    """Get the extrema of a numpy array, ignoring nans

    Arguments:
        this_array: numpy array to get extrema from
        axis: axis to search along

    Returns:
        min: minima of this array
        max: maxima of this array
    """
    extr = (
        numpy.nanmin(this_array, axis=axis),
        numpy.nanmax(this_array, axis=axis))
    return extr

def nanargextrema(this_array, axis=None):
    """Get the location of the extrema, ignoring nans

    Arguments:
        this_array: numpy array to get the extrema location from
        axis: axis to search along

    Returns:
        argmin: argmin of this array
        argmax: argmax of this array
    """
    argextr = (
        numpy.nanargmin(this_array, axis=axis),
        numpy.nanargmax(this_array, axis=axis))
    return argextr

