import pdb

import numpy

import rosgraph_msgs.msg

import bag_records.records

import copy

class Log(bag_records.records.RecordBase):
    """Record for rosgraph_msgs/Log types
    """
    def __init__(self, has_msg_time=True):
        """Constructor

        Arguments:
            has_msg_time: whether this message has a message time field.
                Defaults True

        Returns:
            class instance
        """
        super(Log, self).__init__(has_msg_time)
        self._fields = {
            'bag_time': [],
            'msg_time': [],
            'level': [],
            'name': [],
            'msg': [],
            'file': [],
            'function': [],
            'line': [],
            }
        self._type = rosgraph_msgs.msg.Log
        self._has_msg_time = True

