#!/usr/bin/env python

import rospy

import aircraft_navigation.trajectory_tracker

if __name__ == '__main__':
    rospy.init_node('trajectory_tracker')

    # start the trajectory_tracker
    tracker = aircraft_navigation.trajectory_tracker.TrajectoryTracker()

    # spin while we let everything happen
    rospy.spin()

