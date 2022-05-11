"""
a collection of functions for computing geometric things, right now it contains

rotation matrices
closest point to a line segment or line
"""

import numpy
from math import sin
from math import cos
from math import sqrt
from math import pi

import geometry.quaternion


import warnings

def r2d(angle=1.0):
    """
    convert radian angles to degrees

    Arguments:
        angle: optional (defaults to 1 to return the conversion factor)

    Returns:
        angle_degrees
    """
    warnings.warn('This function has been deprecated, it is now just a call to'
        + ' the numpy rad to degree function', DeprecationWarning)
    return numpy.rad2deg(angle)

def d2r(angle=1.0):
    """
    convert degree angles to radians

    Arguments:
        angle: optional (defaults to 1 to return the conversion factor)

    Returns:
        angle_radians
    """
    warnings.warn('This function has been deprecated, it is now just a call to'
        + ' the numpy degree to rad function', DeprecationWarning)
    return numpy.deg2rad(angle)

def to_unit_vector(this_vector):
    """ Convert a numpy vector to a unit vector

    Arguments:
        this_vector: a (3,) numpy array

    Returns:
        new_vector: a (3,) array with the same direction but unit length
    """
    norm = numpy.linalg.norm(this_vector)
    assert norm > 0.0, "vector norm must be greater than 0"
    if norm:
        return this_vector/numpy.linalg.norm(this_vector)
    else:
        return this_vector

def aircraft_to_ros(x_aircraft):
    """Convert a vector from aircraft to ros coordinates

    The ROS coordinate frame is:
        x: forward
        y: left
        z: up
    Where aircraft standard frame is:
        x: forward
        y: right
        z: down
    This script does the conversion, needed especially when dealing with
    MAVROS which gives everything in ros coordinates. The two conversions are
    actually identical, but both are implemented here

    Arguments:
        x_aircraft: a (3,) numpy array. Also can specify an (n,3) array which
            is interpreted as a sequence of (3,) vectors to rotate

    Returns:
        x_ros: same dimensions as input but in ros coordinate frame
    """
    if x_aircraft.ndim > 1:
        x_ros = numpy.zeros(x_aircraft.shape)
        for idx, x in enumerate(x_aircraft):
            x_ros[i] = aircraft_to_ros(x)
        return x_ros

    x_aircraft[1] *= -1.0
    x_aircraft[2] *= -1.0
    return x_aircraft

def ros_to_aircraft(x_ros):
    """Convert a vector from ros to aircraft coordinates

    Arguments:
        x_ros: a (3,) numpy array. Also can specify an (n,3) array which
            is interpreted as a sequence of (3,) vectors to rotate

    Returns:
        x_aircraft: same dimensions as input but in aircraft coordinate frame
    """
    if x_ros.ndim > 1:
        x_aircraft = numpy.zeros(x_ros.shape)
        for idx, x in enumerate(x_ros):
            x_aircraft[i] = ros_to_aircraft(x)
        return x_aircraft

    x_ros[1] *= -1.0
    x_ros[2] *= -1.0
    return x_ros

def ihat(scale=1.0):
    """Get an x unit vector

    Arguments:
        scale: scale the vector by this value

    Returns:
        ihat: i vector scaled by the specified value
    """
    return numpy.array([1.0, 0.0, 0.0]) * scale

def jhat(scale=1.0):
    """Get an y unit vector

    Arguments:
        scale: scale the vector by this value

    Returns:
        jhat: j vector scaled by the specified value
    """
    return numpy.array([0.0, 0.0, 0.0]) * scale

def khat(scale=1.0):
    """Get an z unit vector

    Arguments:
        scale: scale the vector by this value

    Returns:
        zhat: z vector scaled by the specified value
    """
    return numpy.array([0.0, 0.0, 1.0]) * scale

# This quaternion can represent either the aircraft coordinates in ROS frame
# or vice-versa. (they are 180 degrees rotated around the x axis)
ros_aircraft_quaternion = geometry.quaternion.Quaternion(
    [0.0, numpy.sqrt(2.0)/2.0, numpy.sqrt(2.0)/2.0, 0.0])
