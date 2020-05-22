""" contains line related things
"""
import pdb

import numpy
import scipy.optimize
import shapely.geometry
import spherical_geometry

import copy

import geometry.conversions

def point_line_distance(pt, vertices, is_segment=True):
    """
    compute the minimum vector between a point and a line

    Arguments:
        pt: numpy 1x3 array of the point or a shapely point instance
        vertices: numpy nx3 array of points defining the end of the line segment
            or a shapely LineString or LinearRing instance
        is_segment: optional (defaults true), if false then the point are
            treated as defining an infinitely long line. False is only valid if
            line is a single segment.

    Return:
        r: vector from the point to the line or line segment
        x: numpy 2x3 array of the line segment
        i: the index of the beginning of the closest line segment
    """
    if isinstance(pt, shapely.geometry.Point):
        pt = numpy.array(pt.coords)

    if (isinstance(vertices, shapely.geometry.LineString) or
        isinstance(vertices, shapely.geometry.LinearRing)):
        vertices = numpy.array(vertices.coords)

    if vertices.shape[0] > 2:
        assert is_segment, "only segment computation is valid for lines with\
            more than two points"
        # compute the point as the minimum of this function called recursively
        # over all line segments.
        min_mag_r = numpy.inf
        closest_ind = 0
        r = point_line_distance(pt, vertices[0:2], True)[0]
        for i in range(vertices.shape[0] - 1):
            test_r = point_line_distance(pt, vertices[i:i+2], True)[0]
            if min_mag_r >= numpy.linalg.norm(test_r):
                min_mag_r = numpy.linalg.norm(test_r)
                r = copy.deepcopy(test_r)
                closest_ind = i
        return (r, vertices[closest_ind:closest_ind+2], closest_ind)

    # compute the distance to the infinite line
    line = vertices[-1] - vertices[0]
    line_hat = line/numpy.linalg.norm(line)
    r = (vertices[-1] - pt) - numpy.dot(vertices[-1] - pt, line_hat)*line_hat

    # return it if we aren't dealing with a segment
    if is_segment is False:
        return (r, vertices, 0)

    # check to see if both endpoints lie in the same direction (ie the point
    # does not fall between them...and the minimum distance is simply to one
    # endpoint, if so find the closer endpoint and return a vector to it
    r1 = vertices[-1] - pt
    r2 = vertices[0] - pt
    if numpy.array_equal(numpy.sign(r1 - r), numpy.sign(r2 - r)):
        if numpy.linalg.norm(r1) < numpy.linalg.norm(r2):
            return (r1, vertices, 0)
        else:
            return (r2, vertices, 0)
    return (r, vertices, 0)

def get_point_on_line(datum, vertices, distance, tol=1.0e-4):
    """ Find a point on a line a given distance from a datum.

    Given a line definition, a datum point and distance, find the point which
    lies on the line a given distance from the datum.

    Arguments:
        datum: numpy 3, array of the point or a shapely point instance
        vertices: numpy nx3 array of points defining the end of the line segment
            or a shapely LineString or LinearRing instance
        distance: floating point distance

    Returns:
        positive_point: the point on the line at the prescribed distance
            located in the positive line direction (defined as from the first
            to second vertex)
        negative_point: the point located along the negative line direction
    """

    if isinstance(datum, shapely.geometry.Point):
        datum = numpy.array(datum.coords)

    if (isinstance(vertices, shapely.geometry.LineString) or
        isinstance(vertices, shapely.geometry.LinearRing)):
        vertices = numpy.array(vertices.coords)

    # compute some directions
    direction = numpy.array(vertices[1] - vertices[0], ndmin=2)
    direction /= numpy.linalg.norm(direction)

    if isinstance(distance, numpy.ndarray):
        r = numpy.tile(
            point_line_distance(datum, vertices, False)[0],
            (distance.shape[0], 1))
    else:
        r = numpy.tile(
            point_line_distance(datum, vertices, False)[0],
            (1, 1))

    line_distance = numpy.sqrt(
        -numpy.sum(r * r, axis=1) + numpy.power(distance, 2.0))

    positive_point = numpy.squeeze(r + (direction.T * line_distance).T)
    negative_point = numpy.squeeze(r - (direction.T * line_distance).T)

    return (positive_point, negative_point)

def line_intersections(Xa, ra, Xb ,rb):
    """ Compute the intersection point of two lines

    Arguments:
        Xa: a point on line a
        ra: direction along line a
        Xb: a point on line b
        rb: direction along line b

    Returns:
        ta: scale factor for ra (Xi = Xa + ta * ra)
        tb: scale factor for rb (Xi = Xb + tb * rb)
    """
    assert isinstance(Xa, numpy.ndarray), "Xa must be numpy array"
    assert isinstance(ra, numpy.ndarray), "ra must be numpy array"
    assert isinstance(Xb, numpy.ndarray), "Xb must be numpy array"
    assert isinstance(rb, numpy.ndarray), "rb must be numpy array"

    assert Xa.shape == (3,), "Xa must be (3,)"
    assert ra.shape == (3,), "ra must be (3,)"
    assert Xb.shape == (3,), "Xb must be (3,)"
    assert rb.shape == (3,), "rb must be (3,)"

    normal = numpy.cross(ra, rb)

    if numpy.linalg.norm(normal) < 1.0e-4:
        ta = numpy.inf
        tb = numpy.inf
        return (ta, tb)

    delta_X = Xb - Xa

    if numpy.linalg.norm(delta_X) < 1.0e-4:
        ta = 0.0
        tb = 0.0
        return (ta, tb)

    ta = numpy.cross(delta_X, rb)[2] / normal[2]
    tb = numpy.cross(delta_X, ra)[2] / normal[2]

    return (ta, tb)

def point_arc_distance(pt, vertices, is_segment=True, tol=1.0e-6):
    """Compute the distance between a point and an arc defined by two points

    Derived from the SLERP algorithm, up to an implicit equation that is solved
    implicitly here. All points are assumed to lie on the same sphere, this is
    enforced by normalizing the points before carrying out any calculations.

    Arguments:
        pt: the point, numpy (3,) array, a vector in three space.
        vertices: two points defining the ends of the arc. Should be an
            indexed type whose first two elements are numpy (3,) arrays
        is_segment: optional, defaults True. If true then the arc is
            interpreted as a segment, and if the closest point on the arc
            lies outside the segment, the closest vertex will be returned
        tol: since the implicit equation is solved numerically, this is the
            tolerance to solve it to. defaults to 1.0e-6

    Returns:
        r: the point on the arc closest to the specified point
        x: numpy 2x3 array of the line segment
        i: the index of the beginning of the closest line segment
    """
    assert isinstance(pt, numpy.ndarray), 'pt must be a numpy array'
    assert pt.shape == (3,), 'pt must be (3,)'
    pt = geometry.conversions.to_unit_vector(pt)

    if vertices.shape[0] > 2:
        assert is_segment, "only segment computation is valid for lines with\
            more than two points"
        # compute the point as the minimum of this function called recursively
        # over all line segments.
        min_dist = numpy.inf
        closest_ind = 0
        for i in range(vertices.shape[0] - 1):
            test_r = point_arc_distance(pt, vertices[i:i+2], True, tol)[0]
            test_dist = numpy.abs(spherical_geometry.great_circle_arc.length(
                pt, test_r, degrees=False))
            if min_dist > test_dist:
                min_dist = copy.deepcopy(test_dist)
                r = copy.deepcopy(test_r)
                closest_ind = copy.deepcopy(i)
        return (r, vertices[closest_ind:closest_ind+2], closest_ind)

    assert len(vertices) >= 2, 'vertices must contain at least two elements'
    p0 = geometry.conversions.to_unit_vector(vertices[0])
    p1 = geometry.conversions.to_unit_vector(vertices[1])
    assert isinstance(p0, numpy.ndarray) and isinstance(p0, numpy.ndarray),\
        'vertices must be numpy arrays'
    assert p0.shape == (3,) and p1.shape == (3,), 'vertices must be (3,)'

    # this is a rearranging of the SLERP algorithm which is used to interpolate
    # arcs. SLERP uses a nondimensional parameter in [0,1] to parameterize
    # progress along the arc. We'd like to know the value of that parameter (t)
    # which causes the arc to lie closes to our test point. It's been a while
    # since I did the derivation on this, but this implicit equation should
    # have a root at the closest point on the arc.
    omega = numpy.arccos(p0.dot(p1))
    implicit_equation = lambda x: (
        numpy.cos(x * omega) * p1.dot(pt) -
        numpy.cos((1.0 - x) * omega) * p0.dot(pt))

    # use a scipy solver to find the root of our equation to get the optimal
    # value of the SLERP parameter, t
    t_0 = 0.5
    t_star = scipy.optimize.broyden1(implicit_equation, t_0, f_tol=tol)

    # This evaluates SLERP to determine the closest point on the arc
    p_star = geometry.conversions.to_unit_vector(
        numpy.sin((1.0 - t_star) * omega) / numpy.sin(omega) * p0 +
        numpy.sin(t_star * omega) / numpy.sin(omega) * p1)

    # if we have an infinite line (so that we want to compute the distance to
    # the infinite projection of the arc through the vertices) or if the
    # closest point lies between the two vertices then we're done
    if not is_segment or (t_star <= 1.0 and t_star >= 0.0):
        return (p_star, vertices, 0)

    # if the interpolant is less than zero the closest point is before the
    # first waypoint. Since we have a segment computation, just return it
    if t_star < 0.0:
        return (p0, vertices, 0)

    # since we already trapped the case where t_star is in [0.0, 1.0] then the
    # only remaining case is where the closest point is beyond the final
    # waypoint. Since we're doing the segment thing now, return it
    return (p1, vertices, 0)

def frechet_distance(P, Q):
    """Compute the frechet distance between two discrete curves

    The frechet distance is the minimum length of a line connecting the two
    curves when they are monotonically traversed.

    The algorithm is adapted from:
    Eiter, Thomas, and Mannila, Heikki. Computing discrete Fréchet distance.
    Tech. Report CD-TR 94/64, Information Systems Department,
    Technical University of Vienna, 1994.

    Arguments:
        P: numpy (n,3) array of points defining first curve
        Q: numpy (n,3) array of points defining second curve

    Returns:
        df: frechet distance
        idx: tuple of indices specifying vertices of the points
            i: point on first curve
            j: point on second curve
    """
    def c(ca, P, Q, i, j):
        """Auxiliary function for the recurrence relation
        """
        if not numpy.isnan(ca[i, j]):
            return ca[i, j]
        elif i == 0 and j == 0:
            ca[i,j] = numpy.linalg.norm(P[i] - Q[j])
        elif i > 0 and j == 0:
            ca[i,j] = numpy.nanmax([
                c(ca, P, Q, i - 1, 0),
                numpy.linalg.norm(P[i] - Q[0])])
        elif i == 0 and j > 0:
            ca[i,j] = numpy.nanmax([
                c(ca, P, Q, 0, j - 1),
                numpy.linalg.norm(P[0] - Q[j])])
        elif i > 0 and j > 0:
            ca[i,j] = numpy.nanmax([
                numpy.nanmin([
                    c(ca, P, Q, i - 1, j),
                    c(ca, P, Q, i - 1, j - 1),
                    c(ca, P, Q, i, j - 1)]),
                numpy.linalg.norm(P[i] - Q[j])])
        else:
            ca[i,j] = numpy.inf
        return ca[i, j]

    p = P.shape[0]
    q = Q.shape[0]
    ca = numpy.ones((p,q)) * numpy.nan
    c(ca, P, Q, p - 1, q - 1)
    df = ca[-1,-1]
    ii, jj = numpy.where(ca == df)
    return df, (ii[0], jj[0])
