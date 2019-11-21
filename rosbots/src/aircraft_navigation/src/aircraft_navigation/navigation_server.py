#!/usr/bin/env python
import pdb
"""
"""
import rospy
import actionlib
import numpy
import copy

import spherical_geometry.vector
import spherical_geometry.great_circle_arc

import std_msgs.msg
import mavros_msgs.msg
import mavros_msgs.srv
import sensor_msgs.msg
import geometry_msgs.msg
import aircraft_navigation.msg
import avia_fixed_wing_msgs.msg

import aircraft_navigation.navigator

import geometry.conversions
import geodesy.conversions
import environments.earth
import ros_utilities.conversions

class NavServer(object):
    """ Class handles the server side of the actionlib navigation process. The
    NavServer is the exclusive method for sending waypoint defined acitons to
    the autopilot.
    """
    def __init__(self):
        """ Method initializes errything that the NavServer needs to work.

        Arguments:
        """
        self._debug = True

        self._orbit_watchdog = None

        # make an instance of the actionlib Server
        self.server = actionlib.SimpleActionServer(
            '/aircraft/navigation',
            aircraft_navigation.msg.NavigationAction,
            self.navigate,
            False)

        # instantiate the object that realy does all the work here
        self.navigator = aircraft_navigation.navigator.AircraftNavigator()

        #check the parameter server for information
        self._set_parameters()

        # start the navigation server
        self.server.start()

    def _init_px4(self):
        """Construct publishers and set things up for the px4

        Arguments:
            no arguments

        Returns:
            no returns
        """
        # note that this publisher has little to do with the action,
        # rather it is the vehicle with which the action is carried out
        self._flight_plan_pub = rospy.Publisher(
            '/aircraft/command/flight_plan',
            avia_fixed_wing_msgs.msg.WaypointVector,
            queue_size=5)
        self._wp_command_pub = rospy.Publisher(
            '/aircraft/command/waypoint',
            std_msgs.msg.UInt8,
            queue_size=5)
        self._alt_command_pub = rospy.Publisher(
            '/aircraft/command/altitude',
            std_msgs.msg.Float32,
            queue_size=5)

        self._wp_dict = {
            'climb': 100,
            'explore': 101,
            'target': 103,
            'safe': 90}

        self._next_wp_dict = {
            'climb': 90,
            'explore': 90,
            'target': 90,
            'safe': 90}

        self._tracked_wp = 0

        # define a dict that we'll use to send the waypoint to the proper method
        self._nav_dict = {
            'explore': self.navigator.glide_to_point,
            'climb': self.navigator.glide_to_point,
            'target': self.navigator.glide_to_point,
            'safe': self.navigator.glide_to_point}

        self._position_sub = rospy.Subscriber(
            '/mavros/global_position/global',
            sensor_msgs.msg.NavSatFix,
            self._mavros_position_callback,
            queue_size=1)
        self._velocity_sub = rospy.Subscriber(
            '/mavros/local_position/velocity',
            geometry_msgs.msg.TwistStamped,
            self._mavros_velocity_callback,
            queue_size=1)
        self._wind_sub = rospy.Subscriber(
            '/mavros/wind_estimation',
            geometry_msgs.msg.TwistWithCovarianceStamped,
            self._mavros_wind_callback,
            queue_size=1)

        rospy.loginfo('px4 initialized')

    def navigate(self, nav_action):
        """ Call everything we need to navigate a path and check completion.

        This is a bit messy because we have a lot of navigation related
        functions running around. This does all of the nitty-gritty stuff to
        figure out which we need and call it...

        Arguments:
            nav_action: a Navigation.action goal message. It contains fields
                target_waypoint: a Waypoint message defining the goal
                is_direct_to_orbit: indicates whether the aircraft should pass
                    through the last waypoint before beginning an orbit.

        Returns:
            no returns
        """
        while not self.server.is_active():
            rospy.loginfo('waiting to accept goal')
            rospy.sleep(rospy.Duration(1.0))

        if not nav_action.target_waypoint.description in self._nav_dict.keys():
            rospy.loginfo('rejected, invalid waypoint description')
            self.server.set_aborted()
            return

        if self.navigator.current_location is None:
            rospy.loginfo('rejected, aircraft location is unkown')
            self.server.set_aborted()
            return

        # set the goal waypoint
        goal = nav_action.target_waypoint
        goal_lla = ros_utilities.conversions.vector3_to_numpy(goal.lla)
        if not self.navigator.mission_region.is_point_inside(goal_lla):
            # return a failure if the goal is outside the map
            rospy.loginfo('rejected, destination: {} outside map'.format(
                numpy.rad2deg(goal_lla[0:2])))
            self.server.set_aborted()
            return

        # if we've reached this point, the goal has been accepted
        rospy.loginfo("goal accepted: %s", goal.description)

        goal_X = spherical_geometry.vector.radec_to_vector(
            goal_lla[1], goal_lla[0], degrees=False)

        if self._debug:
            rospy.loginfo(goal_lla)
            rospy.loginfo(self.navigator.current_location)

        angular_distance = spherical_geometry.great_circle_arc.length(
            goal_X, self.navigator.location_X, degrees=False)
        distance = (numpy.abs(angular_distance) *
            environments.earth.constants['r0'])

        #feedback = aircraft_navigation.msg.NavigationFeedback()
        #feedback.distance.x = distance
        #feedback.distance.y = 0.0
        #feedback.distance.z = 0.0
        #self.server.publish_feedback(feedback)

        # call the appropriate navigation method to set the navigator plan
        self._nav_dict[goal.description](goal_lla)

        # start all navigation from Waypoint 150
        wp_num = 150
        wp_list = []
        waypoints = []
        wp_template = {
            'lla': numpy.zeros((3,)),
            'number': 0,
            'next_number': 1,
            'is_preturn': False,
            'orbit_radius': 0.0,
            'orbit_time': 0.0,
            'turn_direction': 1.0,
            }

        for wp in self.navigator.current_plan:
            wp_template['lla'] = wp
            wp_template['number'] = copy.deepcopy(wp_num)
            wp_template['next_number'] = copy.deepcopy(wp_num + 1)
            wp_template['is_preturn'] = True
            wp_template['description'] = ''
            waypoints.append(copy.deepcopy(wp_template))
            wp_list.append(wp_num)
            wp_num += 1

        # if we're going directly to the orbit, this is our last wp
        if nav_action.is_direct_to_orbit:
            waypoints[-1]['orbit_radius'] = goal.radius_orbit
            waypoints[-1]['orbit_time'] = goal.time_orbit
            waypoints[-1]['number'] = self._wp_dict[goal.description]
            waypoints[-1]['next_number'] = self._next_wp_dict[goal.description]
            waypoints[-2]['next_number'] = self._wp_dict[goal.description]
            wp_list[-1] = self._wp_dict[goal.description]
        else:
            # make the last waypoint the destination but orbiting
            wp_template['orbit_radius'] = goal.radius_orbit
            wp_template['orbit_time'] = goal.time_orbit
            wp_template['turn_direction'] = goal.turn_direction
            wp_template['number'] = self._wp_dict[goal.description]
            wp_template['next_number'] = self._next_wp_dict[goal.description]
            waypoints[-1]['next_number'] = self._wp_dict[goal.description]
            wp_list.append(self._wp_dict[goal.description])
            waypoints.append(wp_template)

        # if we're going to the safe point the last waypoint should be a
        # terminal waypoint
        if goal.description == 'safe':
            waypoints[-1]['description'] = 'terminal_waypoint'

        send_waypoints = {
            'px4': self._send_waypoints_px4}
        send_waypoints(waypoints)

        # command the altitude to match the destination altitude now
        altitude_target = std_msgs.msg.Float32()
        altitude_target.data = goal.lla.z

        # publish the altitude target
        self._alt_command_pub.publish(altitude_target)

        # from this point onwards, the aircraft should be flying the plan

        # check completion at 5Hz
        nav_check_rate = rospy.Rate(5)
        self._completed = False
        while not self._completed and not rospy.is_shutdown():
            # check for server crash
            if not self.server.is_active():
                self.server.set_aborted()
                rospy.loginfo('aborted due to server crash')
                return

            # check for preemption
            if self.server.is_preempt_requested():
                self.server.set_preempted()
                return

            # get the distance remaining to the goal location
            # this will also check to make sure we didn't get stuck on an
            # intermediate waypoint. If we're moving away from a waypoint and
            # within some minimum distance of it, we'll kick on to the next one
            if goal.description not in self._nav_tol:
                tolerance = [self._nav_tol['global'], numpy.inf]
            else:
                tolerance = self._nav_tol[goal.description]
            dist, stuck = self.navigator.progress(tolerance)

            # check that we satisfy a "stuck" condition -- that is that we are
            # closer than our navigation performance threshold to a waypoint
            #
            # we check to make sure that we haven't already started on the
            # way to the next waypoing. If we're too close, moving away from a
            # point, and still trying to track it _then_ we'll kick over to the
            # next waypoint. If this is a pixhawk then simply check if we're
            # within the nav tolerance
            if stuck is not None:
                if self._autopilot == 'px4' and stuck < len(wp_list):
                    self._track_wp(wp_list[stuck + 1])

            # report progress through the action server
            feedback = aircraft_navigation.msg.NavigationFeedback()
            feedback.distance.x = dist[0]
            feedback.distance.y = 0.0
            feedback.distance.z = dist[1]
            self.server.publish_feedback(feedback)

            # check tolerances
            if all(dist < self._nav_tol[goal.description]):
                if (waypoints[-1]['orbit_time'] > 0.0):
                    if self._orbit_watchdog is None:
                        self._orbit_watchdog = rospy.Timer(
                            rospy.Duration(waypoints[-1]['orbit_time']),
                            self._finish_orbit,
                            oneshot=True)
                else:
                    self._completed = True
            else:
                nav_check_rate.sleep()

        self._orbit_watchdog = None
        self.server.set_succeeded()
        return

    def _finish_orbit(self, event_data):
        """Watchdog fires when an orbit is completed
        """
        self._completed = True

    def _send_waypoints_px4(self, waypoints):
        """Send waypoints to the px4

        Arguments:
            waypoints: list of waypoint information

        Returns:
            no returns
        """
        waypoint_list = []
        for wp in waypoints:
            new_wp = avia_fixed_wing_msgs.msg.Waypoint()
            new_wp.lla = ros_utilities.conversions.numpy_to_vector3(wp['lla'])
            new_wp.number = wp['number']
            new_wp.next_number = wp['next_number']
            new_wp.turn_direction = wp['turn_direction']
            new_wp.is_preturn = wp['is_preturn']
            new_wp.radius_orbit = wp['orbit_radius']
            new_wp.time_orbit = wp['orbit_time']
            new_wp.description = wp['description']
            waypoint_list.append(new_wp)

        # pack the plan up into a single waypoint string message
        wp_msg = avia_fixed_wing_msgs.msg.WaypointVector()
        wp_msg.waypoints = waypoint_list
        wp_msg.n_waypoints = len(waypoints)
        wp_msg.header.stamp = rospy.Time.now()

        # publish the plan
        self._flight_plan_pub.publish(wp_msg)

        rospy.sleep(rospy.Duration(1.0))

        self._track_waypoint(waypoint_list[1].number)

    def _track_waypoint(self, waypoint_number):
        """Track a waypoint

        Arguments:
            waypoint_number: the id of the waypoint ot track

        Returns:
            no returns
        """
        # command the aircraft to track the first waypoint in the plan
        goto_wp = std_msgs.msg.UInt8()
        goto_wp.data = waypoint_number
        self._wp_command_pub.publish(goto_wp)

    def _current_waypoint_callback(self, wp_msg):
        """ Monitor the currently tracked waypoint

        Arguments:
            wp_msg: avia_fixed_wing_msgs/Waypoint message

        Returns:
            no returns
        """
        self._tracked_wp = wp_msg.number

    def _state_callback(self, state_msg):
        """Callback to aircraft state message

        Arguments:
            state_msg: avia_fixed_wing_msgs/AircraftState message

        Returns:
            no returns
        """
        self.navigator.update_location(
            ros_utilities.conversions.vector3_to_numpy(state_msg.lla))
        self.navigator.update_velocity(
            ros_utilities.conversions.vector3_to_numpy(state_msg.v_gps))
        self.navigator.update_velocity(
            ros_utilities.conversions.vector3_to_numpy(state_msg.wind))

    def _mavros_position_callback(self, gps_fix_msg):
        """Callback to gps position reported by mavros

        Arguments:
            gps_fix_msg: sensor_msgs/NavSatFix message

        Returns:
            no returns
        """
        lla = numpy.array([
            numpy.deg2rad(gps_fix_msg.latitude),
            numpy.deg2rad(gps_fix_msg.longitude),
            gps_fix_msg.altitude])
        self.navigator.update_location(lla)

    def _mavros_velocity_callback(self, velocity_msg):
        """Callback to inertial velocity reported by mavros

        Arguments:
            velocity_msg: geometry_msgs/TwistStamped

        Returns:
            no returns
        """
        v_gps = ros_utilities.conversions.vector3_to_numpy(
            velocity_msg.twist.linear)
        self.navigator.update_velocity(v_gps)

    def _mavros_wind_callback(self, wind_msg):
        """Callback to wind vector reported by mavros

        Arguments:
            wind_msg: geometry_msgs/TwistStamped

        Returns:
            no returns
        """
        wind = ros_utilities.conversions.vector3_to_numpy(
            wind_msg.twist.twist.linear)
        self.navigator.update_wind(wind)

    def _set_parameters(self):
        """Timer callback used for checking the parameter server for changes.
        """
        nav_params = rospy.get_param('/mission/navigation')
        self.navigator.approach_distance = nav_params['upwind_distance']
        self.navigator.min_clearance = nav_params['edge_clearance']
        self._nav_tol = nav_params['waypoint_tolerances']
