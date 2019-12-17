import pdb

import numpy

import geometry.quaternion

import actionlib_msgs.msg

import bag_records.records
import bag_records.std_records

import copy

class GoalID(bag_records.records.RecordBase):
    """Record for actionlib_msgs/GoalID types
    """
    def __init__(self, has_msg_time=False):
        """Constructor

        Arguments:
            has_msg_time: optional bool indicating if this record should have
                a message time. Typically false, but if this is part of a
                message, then it could have a message stamp

        Returns:
            class instance
        """
        super(GoalID, self).__init__(has_msg_time)
        self._fields = {
            'bag_time': [],
            'msg_time': [],
            'stamp': [],
            'id': [],
            }
        self._type = actionlib_msgs.msg.GoalID

class GoalStatus(bag_records.records.RecordBase):
    """Record for actionlib_msgs/GoalStatus types
    """
    def __init__(self, has_msg_time=False):
        """Constructor

        Arguments:
            has_msg_time: optional bool indicating if this record should have
                a message time. Typically false, but if this is part of a
                message, then it could have a message stamp

        Returns:
            class instance
        """
        super(GoalStatus, self).__init__(has_msg_time)
        self._fields = {
            'bag_time': [],
            'msg_time': [],
            'goal_id': GoalID(has_msg_time=has_msg_time),
            'status': [],
            'text': [],
            }
        self._type = actionlib_msgs.msg.GoalStatus

class GoalStatusArray(bag_records.records.RecordBase):
    """Record for actionlib_msgs/GoalStatus types
    """
    def __init__(self, has_msg_time=True):
        """Constructor

        Arguments:
            has_msg_time: optional bool indicating if this record should have
                a message time. Typically false, but if this is part of a
                message, then it could have a message stamp

        Returns:
            class instance
        """
        super(GoalStatusArray, self).__init__(has_msg_time)
        self._fields = {
            'bag_time': [],
            'msg_time': [],
            'status_list': [],
            }
        self._array_types = {
            'status_list': GoalStatus,
            }
        self._type = actionlib_msgs.msg.GoalStatusArray
