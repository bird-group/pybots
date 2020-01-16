import pdb

import numpy

import std_msgs.msg
import robots_common.msg
import geometry_msgs.msg

import bag_records.records

import copy

class Bool(bag_records.records.RecordBase):
    """Record for std_msgs/Bool types
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
        super(Bool, self).__init__(has_msg_time=has_msg_time, interpolate=False)
        self._fields = {'data': [], 'bag_time': [], 'msg_time': []}
        self._type = std_msgs.msg.Bool
        self._has_msg_time = has_msg_time

class Float32(bag_records.records.RecordBase):
    """Record for std_msgs/Float32 types
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
        super(Float32, self).__init__()
        self._fields = {'data': [], 'bag_time': [], 'msg_time': []}
        self._type = std_msgs.msg.Float32
        self._has_msg_time = has_msg_time

class Float32MultiArray(bag_records.records.RecordBase):
    """Record for std_msgs/Float32MultiArray types
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
        super(Float32MultiArray, self).__init__()
        self._fields = {'data': [], 'bag_time': [], 'msg_time': []}
        self._type = std_msgs.msg.Float32MultiArray
        self._has_msg_time = has_msg_time

class Float64(bag_records.records.RecordBase):
    """Record for std_msgs/Float64 types
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
        super(Float64, self).__init__()
        self._fields = {'data': [], 'bag_time': [], 'msg_time': []}
        self._type = std_msgs.msg.Float64
        self._has_msg_time = has_msg_time

class Float64MultiArray(bag_records.records.RecordBase):
    """Record for std_msgs/Float64MultiArray types
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
        super(Float64MultiArray, self).__init__()
        self._fields = {'data': [], 'bag_time': [], 'msg_time': []}
        self._type = std_msgs.msg.Float64MultiArray
        self._has_msg_time = has_msg_time

class Int16(bag_records.records.RecordBase):
    """Record for std_msgs/Int16 types
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
        super(Int16, self).__init__()
        self._fields = {'data': [], 'bag_time': [], 'msg_time': []}
        self._type = std_msgs.msg.Int16
        self._has_msg_time = has_msg_time

class Int16MultiArray(bag_records.records.RecordBase):
    """Record for std_msgs/Int16MultiArray types
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
        super(Int16MultiArray, self).__init__()
        self._fields = {'data': [], 'bag_time': [], 'msg_time': []}
        self._type = std_msgs.msg.Int16
        self._has_msg_time = has_msg_time

class Int32(bag_records.records.RecordBase):
    """Record for std_msgs/Int32 types
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
        super(Int32, self).__init__()
        self._fields = {'data': [], 'bag_time': [], 'msg_time': []}
        self._type = std_msgs.msg.Int32
        self._has_msg_time = has_msg_time

class Int32MultiArray(bag_records.records.RecordBase):
    """Record for std_msgs/Int32MultiArray types
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
        super(Int32MultiArray, self).__init__()
        self._fields = {'data': [], 'bag_time': [], 'msg_time': []}
        self._type = std_msgs.msg.Int32
        self._has_msg_time = has_msg_time

class Int64(bag_records.records.RecordBase):
    """Record for std_msgs/Int64 types
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
        super(Int64, self).__init__()
        self._fields = {'data': [], 'bag_time': [], 'msg_time': []}
        self._type = std_msgs.msg.Int64
        self._has_msg_time = has_msg_time

class Int64MultiArray(bag_records.records.RecordBase):
    """Record for std_msgs/Int64MultiArray types
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
        super(Int64MultiArray, self).__init__()
        self._fields = {'data': [], 'bag_time': [], 'msg_time': []}
        self._type = std_msgs.msg.Int64
        self._has_msg_time = has_msg_time

class Int8(bag_records.records.RecordBase):
    """Record for std_msgs/Int8 types
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
        super(Int8, self).__init__()
        self._fields = {'data': [], 'bag_time': [], 'msg_time': []}
        self._type = std_msgs.msg.Int8
        self._has_msg_time = has_msg_time

class Int8MultiArray(bag_records.records.RecordBase):
    """Record for std_msgs/Int8MultiArray types
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
        super(Int8MultiArray, self).__init__()
        self._fields = {'data': [], 'bag_time': [], 'msg_time': []}
        self._type = std_msgs.msg.Int8
        self._has_msg_time = has_msg_time

class String(bag_records.records.RecordBase):
    """Record for std_msgs/Int64 types
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
        super(String, self).__init__(
            has_msg_time=has_msg_time, interpolate=False)
        self._fields = {'data': [], 'bag_time': [], 'msg_time': []}
        self._type = std_msgs.msg.String
        self._has_msg_time = has_msg_time

    def close(self):
        """Close the bag

        Overrides base method for this, we can't interp it so there's no reason
        to make it a numpy array. We will still array up the stamps

        Arguments:
            no arguments

        Returns:
            no returns
        """
        self._fields['bag_time'] = numpy.array(self._fields['bag_time'])
        if self._has_msg_time:
            self._fields['msg_time'] = numpy.array(self._fields['msg_time'])

class Time(bag_records.records.RecordBase):
    """Record for std_msgs/Time types
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
        super(Time, self).__init__()
        self._fields = {'data': [], 'bag_time': [], 'msg_time': []}
        self._type = std_msgs.msg.Time
        self._has_msg_time = has_msg_time

class UInt16(bag_records.records.RecordBase):
    """Record for std_msgs/UInt16 types
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
        super(UInt16, self).__init__()
        self._fields = {'data': [], 'bag_time': [], 'msg_time': []}
        self._type = std_msgs.msg.UInt16
        self._has_msg_time = has_msg_time

class UInt16MultiArray(bag_records.records.RecordBase):
    """Record for std_msgs/UInt16MultiArray types
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
        super(UInt16MultiArray, self).__init__()
        self._fields = {'data': [], 'bag_time': [], 'msg_time': []}
        self._type = std_msgs.msg.UInt16
        self._has_msg_time = has_msg_time

class UInt32(bag_records.records.RecordBase):
    """Record for std_msgs/UInt32 types
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
        super(UInt32, self).__init__()
        self._fields = {'data': [], 'bag_time': [], 'msg_time': []}
        self._type = std_msgs.msg.UInt32
        self._has_msg_time = has_msg_time

class UInt32MultiArray(bag_records.records.RecordBase):
    """Record for std_msgs/UInt32MultiArray types
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
        super(UInt32MultiArray, self).__init__()
        self._fields = {'data': [], 'bag_time': [], 'msg_time': []}
        self._type = std_msgs.msg.UInt32
        self._has_msg_time = has_msg_time

class UInt64(bag_records.records.RecordBase):
    """Record for std_msgs/UInt64 types
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
        super(UInt64, self).__init__()
        self._fields = {'data': [], 'bag_time': [], 'msg_time': []}
        self._type = std_msgs.msg.UInt64
        self._has_msg_time = has_msg_time

class UInt64MultiArray(bag_records.records.RecordBase):
    """Record for std_msgs/UInt64MultiArray types
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
        super(UInt64MultiArray, self).__init__()
        self._fields = {'data': [], 'bag_time': [], 'msg_time': []}
        self._type = std_msgs.msg.UInt64
        self._has_msg_time = has_msg_time

class UInt8(bag_records.records.RecordBase):
    """Record for std_msgs/UInt8 types
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
        super(UInt8, self).__init__()
        self._fields = {'data': [], 'bag_time': [], 'msg_time': []}
        self._type = std_msgs.msg.UInt8
        self._has_msg_time = has_msg_time

class UInt8MultiArray(bag_records.records.RecordBase):
    """Record for std_msgs/UInt8MultiArray types
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
        super(UInt8MultiArray, self).__init__()
        self._fields = {'data': [], 'bag_time': [], 'msg_time': []}
        self._type = std_msgs.msg.UInt8
        self._has_msg_time = has_msg_time

