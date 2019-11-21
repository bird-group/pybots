""" Navigation computations routines for the sbxc.

Created on Wed, 7 January 2015

@author: birdman
"""
import pdb
import rospy
import rospkg
import numpy
import copy

from shapely.geometry import Point, Polygon
from shapely.ops import cascaded_union
import spherical_geometry.polygon as spherical_polygon
import spherical_geometry.vector
import spherical_geometry.great_circle_arc

import geodesy.conversions
import geometry.conversions
import geometry.rotations
from geodesy.conversions import kml_to_lla
from geometry.conversions import vector3_to_numpy
from geometry.rotations import angle_difference
import environments.earth

import aircraft_navigation.airspace

import sensor_msgs.msg
import geometry_msgs.msg
import avia_fixed_wing_msgs.msg

class AircraftNavigator(object):
    """
    a class for keeping track of aircraft and environmental information needed
    for navigating. it also will do the navigational planning and deal with
    all of the waypoint numbering. Basically this will be the interface between
    actions (ie go thermal here, explore there) and what the piccolo needs to
    know in order to do it fly through these waypoints to wp #120)

    Notionally we'll change to waypoints:
    100: explore
    120: best lift
    140: power climb
    160: safe
    with the intermediate waypoint number available for path planning

    9 January 2015, John Bird
    """

    def __init__(self):
        """ Constructor for the navigation class.

        fetches information regarding the environment from the
        parameter server
        """

        while (not rospy.has_param('/mission/geofence') and
            not rospy.is_shutdown()):
            rospy.loginfo('navigator waiting on geofence')
            rospy.sleep(rospy.Duration(5.0))

        geofence_lat_pts = numpy.array(rospy.get_param('/mission/geofence/lat'))
        geofence_lat = numpy.hstack((geofence_lat_pts, geofence_lat_pts[0]))
        geofence_lon_pts = numpy.array(rospy.get_param('/mission/geofence/lon'))
        geofence_lon = numpy.hstack((geofence_lon_pts, geofence_lon_pts[0]))
        geofence_ref = numpy.flipud(
            numpy.array(rospy.get_param('/mission/geofence/ref_pt')))

        mission_fence = spherical_polygon.SphericalPolygon.from_radec(
            geofence_lon, geofence_lat, geofence_ref, degrees=False)

        self.mission_region =\
            aircraft_navigation.airspace.SphericalEarthAirspace(mission_fence)

        self.location_X = None

        # minimum clearance from boundaries for points in meters
        self.min_clearance = 60.0

        # we need to keep track of the wind vector. eventually I might somehow
        # incorporate a bl model similarish to what we did on the fox.
        self.wind_vector = numpy.zeros((3,))

        # initialize current and sanitized location members
        self.current_location = None
        self.current_velocity = numpy.zeros((3,))

        # keep track of where we're going right now...
        self.current_plan = numpy.zeros((2, 3))

        # keep track of what points have been confirmed by the piccolo
        self._confirmed_points = []
        return

    def glide_to_point(self, destination):
        """ Fly to a point (either an explore point or a climb point).

        our interest here is in simply flying a path to a desired waypoint that
        stays within acceptable bounds

        Arguments:
            destination: shapely Point or 1x3 numpy array specifying the desired
                location in NED map coordinates

        Returns:
            trajectory: not sure yet. maybe a shapely linestring, maybe a
                waypoint message...
        """
        trajectory_lla = numpy.vstack((self.sanitized_location(), destination))
        trajectory_X = numpy.array([
            spherical_geometry.vector.radec_to_vector(
                t[1], t[0], degrees=False)
            for t in trajectory_lla])
        #if not self.mission_region.is_path_inside(trajectory_lla):
        #    return None
        self.current_plan = trajectory_lla
        return trajectory_lla

    def go_to_safe(self, destination):
        """ Fly directly to the safe waypoint with no regard for no fly zones.

        Arguments:
            destination: shapely Point or 1x3 numpy array specifying the desired
                location in NED map coordinates

        Returns:
            trajectory: not sure yet. maybe a shapely linestring, maybe a
                waypoint message...
        """
        return self.go_to_point(destination)
        trajectory = numpy.vstack((self.sanitized_location(), destination))
        self.current_plan = trajectory
        return trajectory

    def _push_point_inside(self, waypoint, min_dist):
        """ Move a point so that it is a minimum distance inside valid airspace.

        push a waypoint inside the airspace so that it is greater than some
        minimum distance from the boundary

        Arguments:
            waypoint: shapely point or numpy 3, array specifying the location
            min_dist: minimum distance from boundary

        Returns:
            waypoint: shapely point adjusted for distance to boundary

        NOTE: the point must be valid in the first place!

        13 January 2015, bird
        """
        if type(waypoint) is Point:
            waypoint = numpy.array(waypoint)
        dist = self.mission_region.nearest_boundary(waypoint)[0]
        if not self.mission_region.is_point_inside(waypoint):
            waypoint += dist * 1.01
        # repeat this process while we are too close, cap at 200 iterations
        cnt = 0
        while numpy.linalg.norm(dist) < min_dist and cnt < 200:
            cnt += 1
            # we need to pull it in for exteriors and push out for holes.
            if self.mission_region.nearest_exterior(waypoint) is not None:
                waypoint -= (
                    dist * (numpy.linalg.norm(dist) - min_dist) / numpy.linalg.norm(dist))
            if self.mission_region.nearest_hole(waypoint) is not None:
                waypoint += (
                    dist * (numpy.linalg.norm(dist) - min_dist) / numpy.linalg.norm(dist))
            dist = self.mission_region.nearest_boundary(waypoint)[0]
        return Point(waypoint)

    def _break_at_intersections(self, trajectory):
        """ Break a trajectory at its intersections with any invalid region.

        the idea is that we'll create a vertex for steering a trajectory every
        time it passes into an invalid region. this will find those
        intersections and create an intermediate point on the trajectory

        Points are added at the mean of all points where the path crosses each
        invalid region.

        !!!!!!!!!!NOTE!!!!!!!!!! This will not work if there is a boundary
        intersection after the first path segment (ie all invalid points must
        lie between the path start and its first waypoint). It places all new
        waypoints in the first path segment sorted by range. With a minor change
        it could work for arbitrary intersection locations with the requirement
        that the initial path has points with monotonically increasing range
        from the bath start

        Arguments:
            trajectory: shapely LineString or nx3 numpy array defining the
                trajectory in map local NED coordinates (m)

        Returns:
            trajectory: nx3 numpy array defining the trajectory with points
                added as required into segments that cross boundaries

        13 January 2015, bird
        """
        # this is unecessary if we already have a legal path
        if self.mission_region.is_path_inside(trajectory):
            return trajectory

        # compute all intersections
        hole_intersects = self.mission_region.hole_intersection(trajectory)
        boundary_intersects = self.mission_region.exterior_intersection(
            trajectory)

        extra_points = []

        # iterate through holes, add a new waypoint at the mean of the
        # intersection points for each
        for (hole, i) in zip(
                hole_intersects, range(len(hole_intersects))):
            # check, as we may not intersect every hole
            if hole:
                points = numpy.array([numpy.array(pt) for pt in hole])
                midpoint = numpy.array(numpy.mean(points, 0), ndmin=2)
                extra_points.append(midpoint)

        # points are added at the midpoint of every two intersections with
        # the exterior boundary (meaning the path must start and end inside)
        boundary_pairs = [(boundary_intersects[i], boundary_intersects[i + 1])
                          for i in range(0, len(boundary_intersects) - 1, 2)]
        for (current_point, next_point) in boundary_pairs:
            extra_points.append(
                (numpy.array(current_point) + numpy.array(next_point)) / 2)

        # sort the extra points by range
        rng = [numpy.linalg.norm(i) for i in extra_points]
        extra_points = numpy.vstack(
            [i for (j, i) in sorted(zip(rng, extra_points), reverse=True)])


        # insert them in the first path segment and return
        return numpy.insert(trajectory, 1, extra_points, axis=0)

    def _legalize_trajectory(self, trajectory):
        """ Modify a trajectory so that it remains in safe airspace.

        This function will compute a trajectory that is legal (stays within the
        bounds and doesn't cross any no fly zones) and reaches the desired point

        NOTE: it is assumed that the destination has already been confirmed to
        lie within valid airspace and that any extra points for steering have
        been added (ie if you have a two point flight plan we can't do anything)

        ok, so this is like, the most naive path planner in all of recorded
        history. At each intersection with a boundary it takes the nearest free
        waypoint (not a start or end) and pushes the waypoint along a vector
        from the centroid of the region it intersects to the mean of all
        intersections between the trajectory and that region.

        Arguments:
            trajectory: shapely LineString instance or nx3 numpy array of
                waypoints. Points are specified in meters NED in the same
                coordinate system that defines the airspace region.

        Returns:
            trajectory: nx3 element numpy array of lat/lon/alt points defining
                the trajectory.

        12 January 2015, John Bird
        """
        cnt = 0
        trajectory = numpy.array(trajectory)
        # pare us down to fewer than 20 points, take the mean of the closest
        # two points iteratively until we are under 20
        while trajectory.shape[0] >= 20:
            pt_dist = numpy.linalg.norm(trajectory[1:] - trajectory[0:-1], axis=1)
            pt_index = numpy.argmin(pt_dist)
            trajectory[pt_index] = numpy.mean(trajectory[pt_index:pt_index+2], 0)
            trajectory = numpy.delete(trajectory, pt_index + 1, 0)
        # give us 200 tries to find a good path, otherwise run until we're
        # entirely inside the mission region
        while not self.mission_region.is_path_inside(trajectory) and cnt < 200:
            cnt += 1
            # compute intersections with holes and boundaries
            hole_intersects = self.mission_region.hole_intersection(trajectory)
            boundary_intersects = self.mission_region.exterior_intersection(
                trajectory)
            # loop through intersections
            for (hole, i) in zip(hole_intersects, range(len(hole_intersects))):
                # check if we intersect this hole
                if hole:
                    points = numpy.array([numpy.array(pt) for pt in hole])
                    midpoint = numpy.array(numpy.mean(points, 0), ndmin=2)
                    # figure out which point we're modifying
                    rng_to_pt = numpy.linalg.norm(trajectory - midpoint, axis=1)
                    pt_index = numpy.argmin(rng_to_pt[1:-1]) + 1
                    # compute a vector from the centroid of the intersected
                    # region to the mean of the intersections
                    centroid = numpy.zeros((1, 3))
                    centroid[0][0:2] = numpy.array(
                        self.mission_region.shape.interiors[i].centroid)
                    xlate_direction = midpoint - centroid
                    delta_hat = (xlate_direction /
                                 numpy.linalg.norm(xlate_direction))
                    xlate_dist = numpy.linalg.norm(points[0] - points[-1])
                    xlate_dist = min(xlate_dist, 0.01 * numpy.linalg.norm(
                        trajectory[0] - trajectory[-1]))
                    delta = delta_hat * xlate_dist
                    # translate the relevant waypoint along this vector
                    trajectory[pt_index] += delta[0]
            if boundary_intersects:
                points = numpy.array(boundary_intersects[0])
                midpoint = numpy.mean(points, 0)
                # figure out which point we're modifying
                rng_to_pt = numpy.linalg.norm(trajectory - midpoint, axis=1)
                pt_index = numpy.argmin(rng_to_pt[1:-1]) + 1
                # compute a vector from the centroid of the intersected
                # region to the mean of the intersections
                centroid = numpy.zeros((1, 3))
                centroid[0][0:2] = numpy.array(self.mission_region.shape.centroid)
                xlate_direction = midpoint - centroid
                delta_hat = xlate_direction / numpy.linalg.norm(xlate_direction)
                xlate_dist = numpy.linalg.norm(points[0] - points[-1])
                xlate_dist = min(xlate_dist, 0.01 * numpy.linalg.norm(
                    trajectory[0] - trajectory[-1]))
                delta = delta_hat * xlate_dist
                # translate the relevant waypoint along this vector
                trajectory[pt_index] -= delta[0]
        return trajectory

    def sanitized_location(self):
        """ Force reported aircraft location to fall within the map.

        Returns the nearest point in the map to the current aircraft location.
        If the aircraft is in the map then it simply reports the location.

        Arguments:
            No arguments

        Returns:
            sanitized_location: 1x3 numpy array giving the location
        """
        return self.current_location

    def progress(self, wp_tol):
        """ compute our progress to the goal

        Callback for nav actions, returns remaining horizontal and vertical
        distance to go to complete the navigation.

        Arguments:
            wp_tol: tolerance on intermediate waypoints. If we're closer than
                this value to a waypoint and moving away from it, then we should
                consider commanding the next waypoint... Should have two elements,
                a horizontal[0] and a vertical[1] tolerance

        Returns:
            togo: 2 element numpy array, linear and vertical distance
            stuck: integer indicating the index of the waypoint we satisfy the
                stuck criteria for... If we're currently tracking that point
                then the nav manager should command the next waypoint. This will
                return None if no waypoint satisfies.
        """
        if self.location_X is None or self.current_location is None:
            return (0, None)

        stuck = None

        goal_X = spherical_geometry.vector.radec_to_vector(
            self.current_plan[-1][1], self.current_plan[-1][0], degrees=False)
        angular_distance = spherical_geometry.great_circle_arc.length(
            goal_X, self.location_X, degrees=False)
        distance = numpy.array([
            numpy.abs(angular_distance) * environments.earth.constants['r0'],
            numpy.abs(self.current_plan[-1][2] - self.current_location[2])])

        return (distance, None)

    def update_location(self, location):
        """ callback to update the location

        Arguments:
            location: numpy array giving lat/lon/alt (rad, m wgs84)

        Returns:
            no returns:
        """
        self.current_location = location
        self.location_X = spherical_geometry.vector.radec_to_vector(
            location[1], location[0], degrees=False)

    def update_velocity(self, velocity):
        """ callback to update the inertial velocity

        Arguments:
            velocity: numpy array giving velocity (NED m/s)

        Returns:
            no returns:
        """
        self.current_velocity = velocity

    def update_wind(self, wind):
        """ callback to update the wind vector

        Arguments:
            velocity: numpy array giving wind vector (NED m/s)

        Returns:
            no returns:
        """
        self.wind_vector = wind

    def waypoint_info_callback(self, wp_number):
        """ callback to update the tracked waypoint

        A waypoint message will be sent every time a waypoint packet is received
        from the piccolo. Here we'll monitor that stream for the reply packet
        confirming that the piccolo has accepted our flight plan.

        Arguments:
            waypoint_number: the identity of the echoed waypoint, should be
                parsed out of the stream by the navigation server.

        Returns:
            no returns
        """
        self._confirmed_points.append(waypoint_number)
        return

    def check_plan_acceptance(self, wp_list):
        """ Check that the flight plan has been accepted

        Arguments:
            wp_list: a list of waypoint numbers in the current plan

        Returns:
            is_accepted: Boolean, True if the piccolo has accepted our plan
        """
        #TODO: currently disabled so that we can work on the px4 stuff
        return True
        for wp in wp_list:
            if wp not in self._confirmed_points:
                return False

        return True

    def clear_confirmed_points(self):
        """ Used to clear our list of confirmed points

        Call this function after planning and before sending the plan out. It
        will clear the list of confirmed points

        Arguments:
            no arguments

        Returns:
            no returns
        """
        self._confirmed_points = []
