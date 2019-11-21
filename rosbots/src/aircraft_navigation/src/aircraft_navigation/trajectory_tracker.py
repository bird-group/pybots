import pdb

import copy

import numpy

import rospy

import communications.ros
import geometry.conversions
import geodesy.conversions
import geodesy.calculations
import robot_control.path_following
import ros_utilities.conversions

import aircraft_resources.message_trackers

import std_msgs.msg
import geometry_msgs.msg
import avia_fixed_wing_msgs.msg

class TrajectoryTracker(object):
    """Track a trajectory
    """
    def __init__(self):
        """Constructor

        Arguments:
            no arguments

        Returns:
            class instance
        """
        self._waypoints = {}

        self._current_waypoint = None

        sub_list = [
            'gps_position', 'gps_velocity', 'imu', 'hud', 'msl_altitude']
        self.aircraft = aircraft_resources.message_trackers.Aircraft(
            namespace='', ap='px4', sub_list=sub_list, sub_energy=False)

        self._init_parameters()
        rospy.loginfo('parameters initialized for {}'.format(rospy.get_name()))

        self._init_publishers()
        rospy.loginfo('publishers initialized for {}'.format(rospy.get_name()))

        self._running = False
        self._command_timer = rospy.Timer(
            rospy.Duration(self._dt), self._service_tracker)

        self._init_subscribers()
        rospy.loginfo('subs initialized for {}'.format(rospy.get_name()))

    def _init_parameters(self, namespace=''):
        """Grab parameters for the tracker from the param server

        Arguments:
            namespace: base param server namespace to use

        Returns:
            no returns
        """
        self._L = communications.ros.wait_for_param(
            namespace + '/control/tracking/L',
            rospy.get_name())
        self._wp_acceptance_radius = communications.ros.wait_for_param(
            namespace + '/control/tracking/waypoint_acceptance_radius',
            rospy.get_name())
        self._dt = communications.ros.wait_for_param(
            namespace + '/control/tracking/command_interval',
            rospy.get_name())
        self._min_orbit_radius = communications.ros.wait_for_param(
            namespace + '/control/tracking/minimum_orbit_radius',
            rospy.get_name())
        self._use_roll_feedback = communications.ros.wait_for_param(
            namespace + '/configuration/use_roll_feedback',
            rospy.get_name())

    def _init_subscribers(self, namespace=''):
        """Start all of the subscribers

        The state subscribers all live in a message tracker, we need to start
        their subscribers up too

        Arguments:
            namespace: optional namespace to put this stuff under

        Returns:
            no returns
        """
        #self.aircraft._init_subcribers()

        self._flight_plan_subscriber = rospy.Subscriber(
            '/aircraft/command/flight_plan',
            avia_fixed_wing_msgs.msg.WaypointVector,
            self._flight_plan_callback,
            queue_size=1)
        self._waypoint_command_subscriber = rospy.Subscriber(
            namespace + '/aircraft/command/waypoint',
            std_msgs.msg.UInt8,
            self._wp_track_callback,
            queue_size=1)

        self._Cl_prime = 0.0
        self._roll_subscriber = rospy.Subscriber(
            namespace + '/estimators/roll/disturbance',
            std_msgs.msg.Float32,
            self._roll_disturbance_callback,
            queue_size=1)

    def _init_publishers(self, namespace=''):
        """Initialize the publishers

        Arguments:
            namespace: optional namespace to put this stuff under

        Returns:
            no returns
        """
        self._acceleration_command_publisher = rospy.Publisher(
            namespace + '/aircraft/command/acceleration',
            geometry_msgs.msg.Vector3Stamped,
            queue_size=1)

    def _flight_plan_callback(self, msg):
        """Get a flight plan

        Arguments:
            msg: avia_fixed_wing_msgs/WaypointVector with waypoint that make
                up this flightplan

        Returns:
            no returns
        """
        for wp in msg.waypoints:
            self._waypoints[wp.number] = wp
        rospy.loginfo('got new flight plan')

    def _wp_track_callback(self, msg):
        """Get command to track a waypoint

        Arguments:
            msg: std_msgs/UInt8 specifying the waypoint

        Returns:
            no returns
        """
        self._wp_command = msg.data
        #TODO: track this waypoint

        if self.check_plan(msg.data):
            rospy.loginfo('received flight plan, tracking waypoint {}'.format(
                msg.data))
            self.track(msg.data)
        else:
            rospy.loginfo('invalid flight plan, waypoint {}'.format(msg.data))

    def check_plan(self, start_point, flight_plan={}):
        """Check if a flight plan is valid from a start waypoint

        Checks that all of the points in a flight plan exist, and that either
        the flight plan closes (returns to a previous waypoint) or the aircraft
        has been directed to circle indefinitely at the end of the plan

        Arguments:
            start_point: the first point to check in the flight plan
            flight_plan: optional, check for this flight plan (existing
                waypoints will be added to this flight plan before checking
                with preference for repeated waypoints given to the new ones)

        Returns:
            is_valid: boolean indicating if the flight plan is valid
        """
        waypoints_visited = []
        idx = start_point

        flight_plan.update(self._waypoints)
        if len(flight_plan) < 1:
            rospy.loginfo('not enough waypoints')
            return False

        while idx not in waypoints_visited:
            waypoints_visited.append(copy.deepcopy(idx))
            if flight_plan[idx].description == 'terminal_waypoint':
                return True
            if flight_plan[idx].next_number not in self._waypoints:
                return False
            idx = flight_plan[idx].next_number

        return True

    def track(self, waypoint_number, start_point=None):
        """Track a waypoint

        Sets a waypoint as the current target, navigates to it, then around the
        flight plan defined by the points linked from this one.

        Arguments:
            waypoint_number: the waypoint we'd like to track

        Returns:
            no returns
        """
        if waypoint_number not in self._waypoints:
            rospy.loginfo(waypoint_number)
            rospy.loginfo(self._waypoints.keys())
        assert waypoint_number in self._waypoints, 'invalid waypoint number'

        waypoint_lla = ros_utilities.conversions.vector3_to_numpy(
            self._waypoints[waypoint_number].lla, ndmin=2)

        if (
            self._waypoints[waypoint_number].description == 'terminal_waypoint'
            or self._waypoints[waypoint_number].time_orbit > 0):
            orbit_radius = max(
                self._waypoints[waypoint_number].radius_orbit,
                self._min_orbit_radius)
            self._tracker = robot_control.path_following.\
                CirclingParkController(
                    waypoint_lla,
                    orbit_radius,
                    self._L,
                    float(self._waypoints[waypoint_number].turn_direction),
                    is_ned=False)
        else:
            if start_point is None:
                start_point = copy.deepcopy(self.aircraft.state.lla)
            end_point = ros_utilities.conversions.vector3_to_numpy(
                self._waypoints[waypoint_number].lla, ndmin=2)
            flight_plan = numpy.vstack((start_point, end_point))

            self._tracker = robot_control.path_following.ParkController(
                flight_plan,
                self._L,
                is_ned=False)

        self._circling = False
        self._loiter_completed = False
        self._tracked_idx = waypoint_number
        self._tracked_waypoint = self._waypoints[waypoint_number]

        self._running = True

    def _service_tracker(self, event_data=None):
        """Callback to timer to generate commands and check progress

        Arguments:
            event_data: optional, information about the timer

        Returns:
            no returns
        """
        if not self._running:
            return
        assert self._tracker is not None, 'tracker not yet specified'

        body_v_ias = numpy.array([self.aircraft.state.v_ias, 0.0, 0.0])
        inertial_v_ias = self.aircraft.state.body_to_inertial(body_v_ias)
        self._tracker.update_state(
            self.aircraft.state.lla[0],
            self.aircraft.state.v_gps)
        acceleration_command = self._tracker.command((True, False, True))

        roll_feedback = numpy.array([0.0, -600.0 * self._Cl_prime, 0.0])
        inertial_roll_feedback = numpy.clip(
            self.aircraft.state.body_to_inertial(roll_feedback), -1.0, 1.0)

        #TODO: uncomment for test
        if self._use_roll_feedback:
            acceleration_command += inertial_roll_feedback

        acceleration_msg = ros_utilities.conversions.numpy_to_vector3stamped(
            acceleration_command)
        acceleration_msg.header.stamp = rospy.Time.now()
        self._acceleration_command_publisher.publish(acceleration_msg)

        if self._check_progress():
            self._running = False
            self.track(self._tracked_waypoint.next_number)

    def _check_progress(self):
        """Check our progress on the current waypoint

        Arguments:
            no arguments

        Returns:
            is_segment_completed: True if we have reached the end of this
                flight segment, either because we are within the acceptance
                radius of the waypoint, or because we have completed the orbit
                time
        """
        if self._circling:
            return self._loiter_completed

        if self._tracked_waypoint.description == 'terminal_waypoint':
            return False

        dx = geodesy.calculations.line_distance(
            self.aircraft.state.lla,
            ros_utilities.conversions.vector3_to_numpy(
                self._waypoints[self._tracked_idx].lla, ndmin=2))

        if dx < self._wp_acceptance_radius:
            if self._tracked_waypoint.time_orbit > 0:
                self._circling = True
                self._loiter_completed = False
                self._loiter_timer = rospy.Timer(
                    rospy.Duration(float(self._tracked_waypoint.time_orbit)),
                    self._loiter_completed_callback,
                    oneshot=True)
                return False
            else:
                return True

    def _loiter_completed_callback(self, event_data=None):
        """Callback from orbit timer

        Fires when the orbit timer has counted down indicating that we're
        done orbiting the required waypoint

        Arguments:
            event_data: information about the timer run

        Returns:
            no returns
        """
        self._loiter_completed = True

    def _roll_disturbance_callback(self, msg):
        """Callback to a message with roll disturbance info

        Arguments:
            msg: std_msgs/Float32

        Returns:
            no returns
        """
        self._Cl_prime = msg.data
