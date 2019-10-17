#!/usr/bin/env python

import pdb
import numpy

import rospy
import rospkg
import sensor_msgs.msg
import communications.ros

class ComputerTemp(object):
    """
    Checks the temperature of the onboard computer, then converts to Kelvin and
    publishes the result.
    """
    
    def __init__(self):
        """ Constructor for the computer temperature class
            
            Arguments:
                none
            
            Returns:
                none
        """
        rospy.init_node('computer_temp')
        
        # Load parameters
        self._init_parameters()
        rospy.loginfo('parameters initialized for {}'.format(rospy.get_name()))

        # Initialize publishers
        self._init_publishers()
        rospy.loginfo('publishers initialized for {}'.format(rospy.get_name()))
        
        # Start up master function and set how often it runs
        self._command_timer = rospy.Timer(
            rospy.Duration(self._dt), self._get_publish_temperature)

    def _init_parameters(self, namespace=''):
        """Pull parameters from parameter server

        Arguments:
            namespace: base parameter server namespace to use

        Returns:
            none
        """
        self._temperature_file_path = communications.ros.wait_for_param(
            namespace + '/avionics/computer/temperature_file_path',
            rospy.get_name())
        self._dt = communications.ros.wait_for_param(
            namespace + '/avionics/computer/temperature_sample_interval',
            rospy.get_name())
        self._temperature_in_C = communications.ros.wait_for_param(
            namespace + '/avionics/computer/temperature_in_C',
            rospy.get_name())
        self._temperature_multiplier = communications.ros.wait_for_param(
            namespace + '/avionics/computer/temperature_multiplier',
            rospy.get_name())
    
    def _init_publishers(self, namespace=''):
        """ Initialize the publishers
        
        Argument:
            namespace: optional namespace
        
        Returns:
            none
        """
        self._temperature_publisher = rospy.Publisher(
            namespace + '/avionics/computer/temperature',
            sensor_msgs.msg.Temperature,
            queue_size=1)
    
    def _get_publish_temperature(self, event_data=None):
        """ Get the temperature and publish it

        Arguments:
            event_data : timer event data structure

        Returns:
            none
        """
        # Open and read the Linux system file that contains the temperature of
        # interest
        self._temperature_file = open(self._temperature_file_path, 'r')
        self._file_contents = self._temperature_file.read()
        self._temperature_file.close()
        # Make sure the value is a float64
        self._temperature = numpy.float64(self._file_contents)
        # Convert measurement to degrees Celsius
        self._temperature = self._temperature / self._temperature_multiplier
        if self._temperature_in_C == False:
            self._temperature = (self._temperature - 32.0) * 5.0 / 9.0
        # Convert measurement to Kelvin
        self._temperature = self._temperature + 273.15
        # Create message
        temperature_msg = sensor_msgs.msg.Temperature()
        temperature_msg.temperature = self._temperature
        temperature_msg.variance = 0.0    # unknown variance
        temperature_msg.header.stamp = rospy.Time.now()
        # Publish message
        self._temperature_publisher.publish(temperature_msg)

if __name__ == '__main__':
    node = ComputerTemp()
    rospy.spin()
