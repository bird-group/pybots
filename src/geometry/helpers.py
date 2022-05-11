""" A collection of geometrical helper things that don't fit in any other module
"""
import itertools
import numpy

def unit_vector(index):
    """ Generate a unit vector

    Arguments:
        index: the index of the unit vector to generate

    Returns:
        n_hat: numpy (3,) array that is zeros except for a one in the position
            indicated by index
    """
    assert index < 3 and index > -1, "index must be between 0 and 2"
    n_hat = numpy.zeros((3,))
    n_hat[index] = 1.0
    return n_hat

def wrap_angle(angle):
    """ Wrap an angle to [-pi, pi]

    Arguments:
        angle: the angle we want to wrap

    Returns:
        wrapped_angle: that angle wrapped to [-pi, pi]
    """
    angle -= (
        float(int((angle + numpy.sign(angle) * numpy.pi) / 2.0 / numpy.pi)) *
        2.0 * numpy.pi
        )
    return angle
