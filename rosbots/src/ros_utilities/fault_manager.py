"""
This module to implement classes and methods for fault management, mostly in
state machines
"""

import rospy

import numpy

import rosgraph_msgs.msg

class FaultManager(object):
    """A class to manage faults
    """
    def __init__(self, topic=None, node_name=None, throttle_interval=10.0):
        """Constructor

        Arguments:
            topic: the topic to publish to. If not specified then faults will
                not be published
            node_name: the name of the node this is running in. If not specified
                then it will be grabeed using rospy.get_name()
            throttle_interval: the minimum interval between throwing faults

        Returns:
            class instance
        """
        if node_name is None:
            self._node_name = rospy.get_name()
        else:
            self._node_name = node_name

        self._throttle_interval = rospy.Duration(throttle_interval)

        if topic is not None:
            self._fault_publisher = rospy.Publisher(
                topic, rosgraph_msgs.msg.Log, queue_size=1)
        else:
            self._fault_publisher = None

        self._timeout = True
        self._watchdog = None

        self._severity_dict = {
            'warn': 4,
            'warning': 4,
            'err': 8,
            'error': 8,
            'fatal': 16}

        self._msg = ''
        self._severity = 4

    def report_fault(self, msg, severity='warn', publish=True, sticky=True):
        """Set a fault

        Arguments:
            msg: the message that should accompany this fault
            severity: identifier for severity. Can be either a string or int
                warning: 4
                error: 8
                fatal: 16
            publish: should we publish or not, defaults true (will not publish
                if this was set up initially without a publisher)
            sticky: should the fault remain in the manager. False will publish
                and clear the fault. True will wait for an explicit clear

        Returns:
            no returns
        """
        if not self._timeout:
            return

        self._msg = msg
        if isinstance(severity, str):
            self._severity = self._severity_dict[severity]
        else:
            self._severity = severity

        if publish and self._fault_publisher is not None:
            fault_report = rosgraph_msgs.msg.Log()
            fault_report.header.stamp = rospy.Time.now()
            fault_report.level = numpy.clip(self._severity, 0, 255)
            fault_report.name = self._node_name
            fault_report.msg = self._msg
            self._fault_publisher.publish(fault_report)

        if not sticky:
            self.clear_fault()

        self._timeout = False
        self._watchdog_timer = rospy.Timer(
            self._throttle_interval, self._throttle_timeout, oneshot=True)

    def _throttle_timeout(self, event_data=None):
        """Callback for the throttling watchdog timer

        Arguments:
            event_data: information about the ros timer event

        Returns:
            no returns
        """
        self._timeout = True

    def clear_fault(self):
        """Clear faults from this manager

        Arguments:
            no arguments

        Returns:
            no returns
        """
        self._msg = ''
        self._severity = 0

    @property
    def fault(self):
        """Get whether there's a current fault or not

        Arguments:
            no arguments

        Returns:
            fault: returns a boolean indicating if there's a fault
        """
        return self._severity > 0

    @property
    def severity(self):
        """Return the severity of the current fault

        Arguments:
            no arguments

        Returns:
            severity: the current fault severity
        """
        return self._severity

    @property
    def msg(self):
        """Return the current fault message

        Arguments:
            no arguments

        Returns:
            msg: the current fault message
        """
        return self._msg
