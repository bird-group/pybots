import pdb

import numpy

import geometry.conversions
import geometry.helpers
import geometry.quaternion
import geodesy.conversions

import environments.earth

import spherical_geometry.vector
import spherical_geometry.great_circle_arc

def line_distance(point_1, point_2, ignore_alt=True):
    """Compute the straight line distance between two points on the earth

    Arguments:
        point_1: numpy (1,3) array giving lat/lon/alt of the first point
        point_2: numpy (1,3) array giving lat/lon/alt of the second point
        ignore_alt: optional, ignore the altitude component, defaults True

    Returns:
        r: distance between the points (m)
    """
    dX = geodesy.conversions.lla_to_ned(point_1, point_2)

    if ignore_alt:
        dX *= numpy.array([1.0, 1.0, 0.0], ndmin=2)

    return numpy.linalg.norm(dX)

def arc_length(point_1, point_2, ignore_alt=True):
    """Compute the great circle arc length between two points on a sphere

    Arguments:
        point_1: numpy (1,3) array giving lat/lon/alt of the first point
        point_2: numpy (1,3) array giving lat/lon/alt of the second point
        ignore_alt: optional, ignore the altitude component, defaults True

    Returns:
        arc_length: the great circle distance
    """
    p1_X = geodesy.conversions.lla_to_vector(point_1)
    p2_X = geodesy.conversions.lla_to_vector(point_2)

    theta = spherical_geometry.great_circle_arc.length(
        p1_X, p2_X, degrees=False)
    return theta

def arc_distance(point_1, point_2, r=None, ignore_alt=True):
    """Compute the great circle distance between two points on a sphere

    Arguments:
        point_1: numpy (1,3) array giving lat/lon/alt of the first point
        point_2: numpy (1,3) array giving lat/lon/alt of the second point
        r: radius of the sphere we're on. Defaults to the earth
        ignore_alt: optional, ignore the altitude component, defaults True

    Returns:
        arc_distance: the great circle distance
    """
    theta = arc_length(point_1, point_2, ignore_alt=ignore_alt)

    if r is None:
        return theta * environments.earth.constants['r0']

    return theta * r

def great_circle_direction(point_1, point_2):
    """Compute the direction for a great circle arc

    Arguments:
        point_1: the starting point of the great circle. The direction will be
            given in a NED frame at this point. Numpy (3,) array in radians, lla
        point_2: the other end of the great circle. can specify a numpy (3,)
            array for a single computation or a numpy (n,3) array for a series
            of computations.

    Returns:
        r_hat: the initial direction of the great circle starting from point_1
    """
    if point_2.ndim > 1:
        directions = numpy.zeros(point_2.shape)
        for idx, coord in enumerate(point_2):
            directions[idx] = great_circle_direction(point_1, coord)
        return directions

    xyz_1 = geodesy.conversions.lla_to_xyz(point_1)
    xyz_2 = geodesy.conversions.lla_to_xyz(point_2)
    khat_xyz = geometry.conversions.to_unit_vector(xyz_1)

    delta = xyz_2 - xyz_1
    delta_hat = geometry.conversions.to_unit_vector(delta)

    r_xyz = numpy.cross(khat_xyz, numpy.cross(delta_hat, khat_xyz))
    r_hat = geodesy.conversions.xyz_to_ned(
        r_xyz + xyz_1, numpy.array(point_1, ndmin=2))[0]
    return geometry.conversions.to_unit_vector(r_hat)

def distance_on_great_circle(start_point, direction, distance):
    """compute the location of a point a specified distance along a great circle

    NOTE: This assumes a spherical earth. The error introduced in the location
    is pretty small (~15 km for a 13000 km path), but it totall screws with
    the altitude. YOU SHOULD NOT USE THE ALTITUDE COMING OUT OF THIS, ESPECIALLY
    IF YOU HAVE ANY MEANGINFUL DISTANCE

    Arguments:
        start_point: the starting point of the great circle. The direction is
            given in a NED frame at this point. Numpy (3,) array in radians, lla
        direction: a NED vector indicating the direction of the great circle
        distance: the length of the great circle arc (m)

    Returns:
        end_point: the end of a great circle path of length <distance> from
            <start_point> with initial <direction>
    """
    start_xyz = geodesy.conversions.lla_to_xyz(start_point)
    direction = geometry.conversions.to_unit_vector(direction)
    delta_xyz = geodesy.conversions.ned_to_xyz(
        direction, numpy.array(start_point, ndmin=2))

    rotation_axis = -geometry.conversions.to_unit_vector(
        numpy.cross(start_xyz, delta_xyz))
    rotation_magnitude = distance / environments.earth.constants['r0']
    rotation_quaternion = geometry.quaternion.Quaternion()
    rotation_quaternion.from_axis_and_rotation(
        rotation_axis, rotation_magnitude)

    end_point_xyz = rotation_quaternion.rot(start_xyz)
    end_point = geodesy.conversions.xyz_to_lla(end_point_xyz)
    return end_point
