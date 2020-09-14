#!/usr/bin/env python3
"""keep some topics alive to deal with a rosbridge/rospy bug"""

import pdb

import roslibpy
import rospy
import rostopic
import std_msgs.msg

import ros_utilities.node

import iriss.msg

class AwfulHackNode(ros_utilities.node.GenericNode):
    """A node which keeps subscriptions and publishes alive to varios topics."""
    def __init__(self, name):
        """Construct the AwfulHackNode object

        Arguments:
            name: string, name of this node

        Returns:
            class instance
        """
        super(AwfulHackNode, self).__init__(name)

    def _init_parameters(self):
        self._publishers = {}
        self._subscribers = {}

        self._roslib_client = roslibpy.Ros(host='localhost', port=9090)
        self._roslib_client.run()

    def _init_timers(self):
        """Construct timers for this node"""
        self._check_topic_timer = rospy.Timer(
            rospy.Duration(1.0),
            self._check_for_topics)


    def _check_for_topics(self, event_data):
        """Check to see if there are any new topics we don't know about"""
        sub_topics = rospy.get_published_topics()

        for topic, msg_type in sub_topics:
            if topic not in self._publishers:
                print('registering publisher for {}'.format(topic))
                self._publishers[topic] = roslibpy.Topic(
                    self._roslib_client,
                    topic,
                    self._roslib_client.get_topic_type(topic))
                self._publishers[topic].advertise()
            if topic not in self._subscribers:
                print('registering subscriber for {}'.format(topic))
                self._subscribers[topic] = roslibpy.Topic(
                    self._roslib_client,
                    topic,
                    self._roslib_client.get_topic_type(topic))
                self._subscribers[topic].subscribe(self._null_callback)

    def _null_callback(self, msg):
        """A null message callback"""
        return

    def cleanup(self):
        """cleanup the roslib client"""
        self._roslib_client.terminate()

if __name__ == '__main__':
    node = AwfulHackNode('terrible_horrible_no_good_very_bad_node')
    rospy.spin()
    node.cleanup()
