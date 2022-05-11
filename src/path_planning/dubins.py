import pdb

import numpy

import geometry.lines
import geometry.rotations

import matplotlib.pyplot as plt

class _Primitive(object):
    """A superclass for dubins path primitives, should not be directly invoked
    """
    def __init__(self):
        self._words = {-1: 'R', 0: 'S', 1: 'L', numpy.nan: 'N'}
        self._direction = numpy.nan
        self._X0 = numpy.zeros((3,)) * numpy.nan
        self._Xf = numpy.zeros((3,)) * numpy.nan

    @property
    def word(self):
        """Property getter for the word describing this primitive

        Arguments:
            no arguments

        Returns:
            description: word describing this element
        """
        return self._words[self._direction]

    @property
    def start_point(self):
        """Get the start point of the segment

        Arguments:
            no arguments

        Returns:
            X0: point where this element starts
        """
        return self._X0

    @property
    def end_point(self):
        """Get the end point of the segment

        Arguments:
            no arguments

        Returns:
            Xf: point where this element ends
        """
        return self._Xf

class Null(_Primitive):
    """A placeholder class that does not represent a valid primitive.

    This is used by the planner to simplify idenfiying the optimal dubins path
    trajectory as it can be added to an array of solutions representing an
    infinitely costly no solution
    """
    def __init__(self):
        super(Null, self).__init__()

    @property
    def length(self):
        """Property getter for path length of the primitive

        Arguments:
            no arguments

        Returns:
            l: path length
        """
        return numpy.inf

    def point_on_path(self, distance):
        """Get a point on the path defined by the primitive

        Arguments:
            distance: distance along the path to get the point

        Returns:
            point: returns the point on the path, for a null primitive returns
                a (3,) array of nan
        """
        return numpy.ones((3,)) * numpy.nan

class Straight(_Primitive):
    """Class for dubins straight primitive
    """
    def __init__(self, X0, Xf):
        """Constructor

        Arguments:
            X0: numpy (3,) array giving start location
            Xf: numpy (3,) array giving end location

        Returns:
            class instance
        """
        super(Straight, self).__init__()
        self._X0 = X0
        self._Xf = Xf
        self._line = numpy.stack((self._X0, self._Xf))
        self._direction = 0

    @property
    def length(self):
        """Property getter for path length of the primitive

        Arguments:
            no arguments

        Returns:
            l: path length
        """
        return numpy.linalg.norm(self._Xf - self._X0)

    @property
    def center(self):
        """Getter for center of curvature of the path

        This is a placeholder so that the center of curvature can be queried
        and plotted for all elements in a dubins path without having to check
        if the element is an arc or straight
        """
        return numpy.ones((3,)) * numpy.nan

    def point_on_path(self, distance):
        """Get a point on the path defined by the primitive

        Arguments:
            distance: distance along the path to get the point

        Returns:
            points: numpy (n,3) array specifiying the point located on this
                primitive the specified distance from the start. Will clip to
                the terminal point of the primitive or to the initial point for
                negative distance
        """
        distance = numpy.array(distance, ndmin=1)
        points = numpy.zeros((distance.shape[0], 3))
        points[distance >= self.length] = self._Xf
        points[distance <= 0] = self._X0

        unclipped_mask = numpy.logical_and(distance < self.length, distance > 0)
        if not numpy.any(unclipped_mask):
            return points

        points[unclipped_mask] = geometry.lines.get_point_on_line(
            self._X0, self._line, distance[unclipped_mask])[0] + self._X0
        return points

class Arc(_Primitive):
    """Class for dubins arc primitive
    """
    def __init__(self, X0, Xf, r, V0):
        """Constructor

        Arguments:
            X0: numpy (3,) array giving start location
            Xf: numpy (3,) array giving end location
            r: arc radius. sign indicates direction (left positive)
            V0: numpy (3,) array giving initial direction

        Returns:
            class instance
        """
        super(Arc, self).__init__()
        self._X0 = X0
        self._Xf = Xf
        self._r = numpy.abs(r)
        self._direction = numpy.sign(r)

        # the circle center is perpendicular to the initial velocity vector
        rk = numpy.array([0.0, 0.0, self._direction])
        V0_hat = V0 / numpy.linalg.norm(V0)
        self._Xc = numpy.cross(rk, V0_hat) * self._r + self._X0

        # compute the angle of the arc from X0 to Xf
        l = numpy.linalg.norm(self._X0 - self._Xf)
        theta = numpy.arcsin(l / self._r / 2.0) * 2.0
        # if the vector from the initial to final point is in the opposite
        # direction as the initial velocity vector then we're going the long
        # way around the arc
        if numpy.dot(self._Xf - self._X0, V0_hat) > 0:
            self._arc_angle = theta * self._direction
        else:
            self._arc_angle = (2.0 * numpy.pi - theta) * self._direction

    @property
    def length(self):
        """Property getter for length of the path

        Arguments:
            no arguments

        Returns:
            l: path length
        """
        return numpy.abs(self._arc_angle * self._r)

    @property
    def center(self):
        """Getter for center of curvature of the path

        Arguments:
            no arguments

        Returns:
            center_point: location of the center of this arc
        """
        return self._Xc

    def point_on_path(self, distance):
        """Get a point on the path defined by the primitive

        Arguments:
            distance: distance along the path to get the point

        Returns:
            points: numpy (n,3) array specifiying the point located on this
                primitive the specified distance from the start. Will clip to
                the terminal point of the primitive or to the initial point for
                negative distance
        """
        distance = numpy.array(distance, ndmin=1)
        points = numpy.zeros((distance.shape[0], 3))
        points[distance >= self.length] = self._Xf
        points[distance <= 0] = self._X0

        unclipped_mask = numpy.logical_and(distance < self.length, distance > 0)
        if not numpy.any(unclipped_mask):
            return points

        theta = distance / self._r * self._direction
        x0 = self._X0 - self._Xc
        if isinstance(distance, numpy.ndarray):
            if theta.shape == (0,):
                return numpy.zeros((0,3))
            rmatrix = numpy.stack([geometry.rotations.zrot(-t) for t in theta])
        else:
            rmatrix = geometry.rotations.zrot(theta)
        points[unclipped_mask] = (
            numpy.squeeze(rmatrix[unclipped_mask].dot(x0)) + self._Xc)
        return points

class Path(object):
    """A class for dubins paths
    """
    def __init__(self, elements):
        """Constructor

        Arguments:
            elements: iterable of dubins path primitives composing the path

        Returns:
            class instance
        """
        self._elements = elements

    @property
    def length(self):
        """Property getter for path length of the primitive

        Arguments:
            no arguments

        Returns:
            l: path length
        """
        return numpy.sum([e.length for e in self._elements])

    @property
    def word(self):
        """Property getter for the word describing this primitive

        Arguments:
            no arguments

        Returns:
            description: word describing this element
        """
        return ''.join([e.word for e in self._elements])

    def point_on_path(self, distance):
        """Find a point along the path at a specified distance

        Arguments:
            distance: distance along the path to get the point

        Returns:
            points: numpy (n,3) array specifiying the point located on this
                primitive the specified distance from the start. Will clip to
                the terminal point of the primitive or to the initial point for
                negative distance
        """
        distance = numpy.array(distance, ndmin=1)

        points = numpy.zeros((distance.shape[0], 3))
        points[distance >= self.length] = self._elements[-1].end_point
        points[distance <= 0] = self._elements[0].start_point

        unclipped_mask = numpy.logical_and(distance < self.length, distance > 0)
        if not numpy.any(unclipped_mask):
            return points

        # we need to identify which distance terminate in each element so we
        # keep a running distance which is incremented for each element and
        # compare that against the distances
        d = numpy.zeros(distance.shape)
        d = 0.0
        for e in self._elements:
            to_go = distance - d

            idx_this_segment = numpy.logical_and(to_go < e.length, to_go > 0)
            idx_next_segment = to_go > e.length

            d += e.length

            points[idx_this_segment] = e.point_on_path(to_go[idx_this_segment])
        return points

def circle_tangents(points, radius, turn_directions):
    """Find tangent point for elements joining start and goal points

    Note that it is possible that some configurations may not be solvable. In
    this case all returns will be None. This code is adapted from the
    circle_tangents.m and ThreeCircleTangents.m files provided by Andrew and
    Katiee.

    Arguments:
        points: numpy (2,3) giving the centers of the initial and final arcs on
            the dubins path
        radius: the turn radius of the arcs
        turn_directions:  (2,) element iterable giving the sign of the turn
            direction for those arcs

    Returns:
        first_tangent: the location of the tangent between the first arc and
            the middle element
        second_tangent: the location of the tangent between the middle segment
            and the last arc
        second_center: the location of the center of curvature of the middle
            element. Will return None if the middle segment is a line
    """
    center_line = numpy.diff(points, axis=0)[0]
    D = numpy.linalg.norm(center_line)
    y = numpy.arctan2(center_line[1], center_line[0])

    if turn_directions[1] == 0:
        idx = numpy.array([True, False, True])
        r4 = numpy.abs(
            numpy.sum(-turn_directions[idx] * numpy.array([1, -1]) * radius))

        if r4 > D:
            return None, None, None

        theta = y - turn_directions[0] * numpy.arccos(r4 / D)
        center_point = points[0] + r4 * numpy.array([
            numpy.cos(theta), numpy.sin(theta), 0.0])
        first_tangent = points[0] + radius * numpy.array([
            numpy.cos(theta), numpy.sin(theta), 0.0])
        second_tangent = first_tangent + (points[1] - center_point)
        second_center = None
    elif D / 4.0 / radius > 1.0:
        first_tangent = None
        second_tangent = None
        second_center = None
    else:
        theta = y + numpy.arccos(D / 4.0 / radius) * -turn_directions[1]
        first_tangent = points[0] + radius * numpy.array([
            numpy.cos(theta), numpy.sin(theta), 0.0])
        center_point = points[0] + 2.0 * radius * numpy.array([
            numpy.cos(theta), numpy.sin(theta), 0.0])
        second_tangent = center_point + (points[1] - center_point) / 2.0
        second_center = center_point

    return first_tangent, second_tangent, second_center

def plan(start, goal, r):
    """Create a dubins plan from a start point to a goal

    Arguments:
        start: list with elements
            location: numpy (3,) array giving start point
            direction: direction of start
        goal: list with elements
            location: numpy (3,) array giving end point
            direction: direction of end
        r: turn radius of vehicle

    Returns:
        path: a Path instance
    """
    start_point = start[0]
    start_psi = start[1]
    goal_point = goal[0]
    goal_psi = goal[1]

    # direction (LSL, LSR, RSL, RSR, LRL, RLR)
    directions = numpy.array([
        [1, 1, -1, -1, 1, -1],
        [0, 0, 0, 0, -1, 1],
        [1, -1, 1, -1, 1, -1]
        ]).T

    paths = []
    for direction in directions:
        # build direction vectors
        v0_hat = numpy.array([numpy.cos(start_psi), numpy.sin(start_psi), 0.0])
        vf_hat = numpy.array([numpy.cos(goal_psi), numpy.sin(goal_psi), 0.0])
        first_r_k = numpy.array([0.0, 0.0, r * direction[0]])
        middle_r_k = numpy.array([0.0, 0.0, r * direction[1]])
        last_r_k = numpy.array([0.0, 0.0, r * direction[2]])

        # compute circle centers
        first_turn_point = start_point + numpy.cross(first_r_k, v0_hat)
        last_turn_point = goal_point + numpy.cross(last_r_k, vf_hat)

        # get points of tangency for the arcs
        points = numpy.stack([first_turn_point, last_turn_point])
        tangents = circle_tangents(points, r, direction)

        if tangents[0] is None:
            paths.append(Null())
            continue

        # construct the elements comprising the path
        first_arc = Arc(start_point, tangents[0], r * direction[0], v0_hat)
        middle_direction = numpy.cross(
            tangents[0] - first_arc.center, -first_r_k)
        if direction[1] == 0:
            middle_segment = Straight(tangents[0], tangents[1])
            end_direction = tangents[1] - tangents[0]
        else:
            middle_segment = Arc(
                tangents[0], tangents[1], r * direction[1], middle_direction)
            end_direction = numpy.cross(
                tangents[1] - middle_segment.center, -middle_r_k)
        last_arc = Arc(tangents[1], goal_point, r * direction[2], end_direction)
        paths.append(Path([first_arc, middle_segment, last_arc]))

    # choose shortest path from the six options
    path_length = numpy.array([p.length for p in paths])
    idx = numpy.nanargmin(path_length)
    return paths[idx]

if __name__ == '__main__':
    start_pt = numpy.hstack([numpy.random.rand(2), 0.0])
    start_heading = numpy.random.rand() * numpy.pi * 2.0
    goal_pt = numpy.hstack([numpy.random.rand(2), 0.0])
    goal_heading = numpy.random.rand() * numpy.pi * 2.0

    start = [start_pt, start_heading]
    goal = [goal_pt, goal_heading]
    r = 0.1

    path_star = plan(start, goal, r)
    s = numpy.linspace(0, path_star.length, 200)
    pts = path_star.point_on_path(s)

    v0_hat = numpy.array(
        [numpy.cos(start_heading), numpy.sin(start_heading), 0.0])
    vf_hat = numpy.array(
        [numpy.cos(goal_heading), numpy.sin(goal_heading), 0.0])

    plt.scatter(start_pt[0], start_pt[1], marker='x', c='b')
    plt.scatter(goal_pt[0], goal_pt[1], marker='x', c='r')
    plt.quiver(
        start_pt[0], start_pt[1], v0_hat[0], v0_hat[1], color='b')
    plt.quiver(
        goal_pt[0], goal_pt[1], vf_hat[0], vf_hat[1], color='r')
    plt.plot(pts[:,0], pts[:,1], 'k')
    color = ['b', 'c', 'r']
    for c, e in zip(color, path_star._elements):
        s = numpy.linspace(0, e.length, 100)
        pts = e.point_on_path(s)
        plt.scatter(e._X0[0], e._X0[1], c=c, marker='o', facecolors='none')
        plt.scatter(e._Xf[0], e._Xf[1], c=c, marker='+')
        plt.scatter(e.center[0], e.center[1], c=c, marker='d')
        plt.plot(pts[:,0], pts[:,1], '--', c=c)

    plt.title(path_star.word)
    plt.grid()
    plt.axis('equal')
    plt.show()
