#!/usr/bin/env python

import rospy

import aircraft_navigation.navigation_server

if __name__ == '__main__':
    rospy.init_node('navigator')

    assert rospy.has_param('/communication/autopilot'), 'autopilot type not specified'

    autopilot_type = rospy.get_param('/communication/autopilot')
    # start the navigation server
    action_server = aircraft_navigation.navigation_server.NavServer(
        autopilot_type)

    # spin while we let everything happen
    rospy.spin()
