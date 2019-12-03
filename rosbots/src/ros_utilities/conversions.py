import numpy

import rospy

import geometry.spline_model
import rosbots.msg

import geometry_msgs

import numpy

def msg_to_dict(msg):
    """ Converts a rospy message to a dictionary of stuff

    ROS messages don't have a __dict__ attribute so I wrote this lil guy to
    convert them into dictionaries. Any ros messages encountered along the way
    will also be converted so we'll end up with a dictionary of dictionaries

    Arguments:
        msg: the ros message to convert

    Returns:
        msg_dict: the resulting dictionary
    """
    # Apparently classes made in C don't have a __dict__ attribute (which will
    # return the object's attributes as a dictionary. ROS Messages do however
    # have a __slots__ attribute which lists all of the names of its properties
    # we'll loop over them. If one of the properties has __slots__ it must be
    # a nested method so we'll recursively call this function to create subdicts
    msg_dict = {}
    for varname in msg.__slots__:
        var = getattr(msg, varname)
        if '__slots__' in dir(var):
            msg_dict[varname] = msg_to_dict(getattr(msg, varname))
        else:
            msg_dict[varname] = getattr(msg, varname)
    return msg_dict

def spline_to_msg(spline):
    """Create a spline message from a spline

    Arguments:
        spline: SplineModel object to convert

    Returns:
        spline_msg: rosbots/SplineModel message
    """
    spline_msg = rosbots.msg.SplineModel()

    spline_msg.knots = spline._spline.knots
    spline_msg.coefficients = spline._spline.coords
    spline_msg.covariance = spline._P.flatten()
    spline_msg.order = spline._spline.order
    return spline_msg

def msg_to_spline(msg):
    """Create a spline model from a message

    Arguments:
        msg: rosbots/SplineModel message

    Returns:
        spline: SplineModel object
    """
    P = numpy.reshape(
        msg.covariance, (len(msg.coefficients), len(msg.coefficients)))
    bknots = (msg.knots[:msg.order], msg.knots[-msg.order:])
    spline = geometry.spline_model.SplineModel(
        msg.knots[msg.order:-msg.order],
        msg.order,
        msg.coefficients,
        P=P,
        boundary_knots=bknots)
    return spline

def vector3stamped_to_numpy(v3, ndmin=1):
    """ Convert a geometry_msgs/Vector3Stamped to numpy array

    Arguments:
        v3: ROS geometry_msgs/Vector3Stamped message

    Returns:
        np_vector: 3, numpy vector
    """
    assert hasattr(v3,'vector'), "must input a Vector3Stamped-like structure!"
    return vector3_to_numpy(v3.vector, ndmin)

def numpy_to_vector3stamped(np_vector):
    """ Convert a numpy array to a Vector3Stamped

    Arguments:t.
        np_vector: 3, numpy array

    Returns:
        v3: Vector3Stamped message
    """
    assert type(np_vector) is numpy.ndarray, "np_vector must be numpy array"
    assert np_vector.shape == (3,), "must be 3, numpy array"
    v3 = geometry_msgs.msg.Vector3Stamped()
    v3.vector = numpy_to_vector3(np_vector)
    return v3

def vector3_to_numpy(this_vector3, ndmin=1):
    """Convert a vector3 message to a numpy array

    Notes: this is also used by the point32_to_numpy method

    Arguments:
        this_vector3: a geometry_msgs/Vector3 message to turn into a numpy array
        ndmin: minimum dimension of resulting array, defaults to 1

    Returns:
        this_numpy: a numpy array with a 3 element axis containing the data from
            the vector3 message
    """
    this_numpy = numpy.array(
        [this_vector3.x, this_vector3.y, this_vector3.z], ndmin=ndmin)
    return this_numpy

def point32_to_numpy(this_point32, ndmin=1):
    """Convert a poing message to a numpy array

    Notes: this aliases the vector3_to_numpy method internally

    Arguments:
        this_point32: a geometry_msgs/Point32 message to turn into a numpy array
        ndmin: minimum dimension of resulting array, defaults to 1

    Returns:
        this_numpy: a numpy array with a 3 element axis containing the data from
            the Point32 message
    """
    return vector3_to_numpy(this_point32, ndmin)

def point_to_numpy(this_point32, ndmin=1):
    """Convert a point message to a numpy array

    Notes: this aliases the vector3_to_numpy method internally

    Arguments:
        this_point: a geometry_msgs/Point message to turn into a numpy array
        ndmin: minimum dimension of resulting array, defaults to 1

    Returns:
        this_numpy: a numpy array with a 3 element axis containing the data from
            the Point message
    """
    return vector3_to_numpy(this_point, ndmin)

def _numpy_to_3element(this_numpy, message_constructor):
    """Backend for converting a numpy array into a three element message

    Arguments:
        this_numpy: a numpy array having one 3 element dimension (so that it
            can be squeezed to (3,) size) to populate a message from
        message_constructor: handle to the constructor for the desired message

    Returns:
        this_message: a message of the appropriate type with the numpy array
            populating its .x .y and .z properties
    """
    this_numpy = numpy.squeeze(this_numpy)
    this_message = message_constructor()
    this_message.x = this_numpy[0]
    this_message.y = this_numpy[1]
    this_message.z = this_numpy[2]
    return this_message

def numpy_to_vector3(this_numpy):
    """Convert a numpy array into a geometry_msgs/Vector3

    Arguments:
        this_numpy: a numpy array having one 3 element dimension (so that it
            can be squeezed to (3,) size) to populate a message from

    Returns:
        this_vector3: geometry_msgs/Vector3 message with data from this_numpy
    """
    return _numpy_to_3element(this_numpy, geometry_msgs.msg.Vector3)

def numpy_to_point32(this_numpy):
    """Convert a numpy array into a geometry_msgs/Point32

    Arguments:
        this_numpy: a numpy array having one 3 element dimension (so that it
            can be squeezed to (3,) size) to populate a message from

    Returns:
        this_point32: geometry_msgs/Point32 message with data from this_numpy
    """
    return _numpy_to_3element(this_numpy, geometry_msgs.msg.Point32)

def numpy_to_point(this_numpy):
    """Convert a numpy array into a geometry_msgs/Point32

    Arguments:
        this_numpy: a numpy array having one 3 element dimension (so that it
            can be squeezed to (3,) size) to populate a message from

    Returns:
        this_point32: geometry_msgs/Point32 message with data from this_numpy
    """
    return _numpy_to_3element(this_numpy, geometry_msgs.msg.Point)
