import pdb

import numpy

import std_msgs.msg
import robots_common.msg
import geometry_msgs.msg
import mavros_msgs.msg

import bag_records.records
import bag_records.std_records
import bag_records.geometry_records

import copy

class Mavlink(bag_records.records.RecordBase):
    """Record for mavros_msgs/Mavlink types
    """
    def __init__(self, has_msg_time=True):
        """Constructor

        Arguments:
            has_msg_time: whether this message has a message time field.
                Defaults True

        Returns:
            class instance
        """
        super(Mavlink, self).__init__(has_msg_time)
        self._fields = {
            'bag_time': [],
            'msg_time': [],
            'framing_status': [],
            'magic': [],
            'len': [],
            'incompat_flags': [],
            'compat_flags': [],
            'seq': [],
            'sysid': [],
            'compid': [],
            'msgid': [],
            'checksum': [],
            'payload64': [],
            'signature': [],
            }
        self._type = mavros_msgs.msg.Mavlink

class Altitude(bag_records.records.RecordBase):
    """Record for mavros_msgs/Altitude types
    """
    def __init__(self, has_msg_time=True):
        """Constructor

        Arguments:
            has_msg_time: whether this message has a message time field.
                Defaults True

        Returns:
            class instance
        """
        super(Altitude, self).__init__(has_msg_time)
        self._fields = {
            'bag_time': [],
            'msg_time': [],
            'monotonic': [],
            'amsl': [],
            'local': [],
            'relative': [],
            'terrain': [],
            'bottom_clearance': [],
            }
        self._type = mavros_msgs.msg.Altitude

class ExtendedState(bag_records.records.RecordBase):
    """Record for mavros_msgs/ExtendedState types
    """
    def __init__(self, has_msg_time=True):
        """Constructor

        Arguments:
            has_msg_time: whether this message has a message time field.
                Defaults True

        Returns:
            class instance
        """
        super(ExtendedState, self).__init__(has_msg_time)
        self._fields = {
            'bag_time': [],
            'msg_time': [],
            'vtol_state': [],
            'landed_state': [],
            }
        self._type = mavros_msgs.msg.ExtendedState

class HiLActuatorControls(bag_records.records.RecordBase):
    """Record for mavros_msgs/HilActuatorControls types
    """
    def __init__(self, has_msg_time=True):
        """Constructor

        Arguments:
            has_msg_time: whether this message has a message time field.
                Defaults True

        Returns:
            class instance
        """
        super(HiLActuatorControls, self).__init__(has_msg_time)
        self._fields = {
            'bag_time': [],
            'msg_time': [],
            'controls': [],
            'mode': [],
            'uint64': [],
            }
        self._type = mavros_msgs.msg.HiLActuatorControls

class ManualControl(bag_records.records.RecordBase):
    """Record for mavros_msgs/ManualControl types
    """
    def __init__(self, has_msg_time=True):
        """Constructor

        Arguments:
            has_msg_time: whether this message has a message time field.
                Defaults True

        Returns:
            class instance
        """
        super(ManualControl, self).__init__(has_msg_time)
        self._fields = {
            'bag_time': [],
            'msg_time': [],
            'x': [],
            'y': [],
            'z': [],
            'r': [],
            'buttons': [],
            }
        self._type = mavros_msgs.msg.ManualControl

class Waypoint(bag_records.records.RecordBase):
    """Record for mavros_msgs/Waypoint types
    """
    def __init__(self, has_msg_time=False):
        """Constructor

        Arguments:
            has_msg_time: bool indicating if this has a message time field
                Defaults false

        Returns:
            class instance
        """
        super(Waypoint, self).__init__(has_msg_time)
        self._fields = {
            'bag_time': [],
            'msg_time': [],
            'command': [],
            'is_current': [],
            'autocontinue': [],
            'param1': [],
            'param2': [],
            'param3': [],
            'param4': [],
            'x_lat': [],
            'y_long': [],
            'z_alt': [],
            }
        self._type = mavros_msgs.msg.Waypoint

class WaypointList(bag_records.records.RecordBase):
    """Record for mavros_msgs/WaypointList types
    """
    def __init__(self, has_msg_time=False):
        """Constructor

        Arguments:
            has_msg_time: bool indicating if this has a message time field
                defaults false

        Returns:
            class instance
        """
        super(WaypointList, self).__init__(has_msg_time)
        self._fields = {
            'bag_time': [],
            'msg_time': [],
            'waypoints': Waypoint(has_msg_time=has_msg_time),
            }
        self._type = mavros_msgs.msg.Waypoint

class RCIn(bag_records.records.RecordBase):
    """Record for mavros_msgs/RCIn types
    """
    def __init__(self, has_msg_time=True):
        """Constructor

        Arguments:
            has_msg_time: whether this message has a message time field.
                Defaults True

        Returns:
            class instance
        """
        super(RCIn, self).__init__(has_msg_time)
        self._fields = {
            'bag_time': [],
            'msg_time': [],
            'rssi': [],
            'channels': [],
            }
        self._type = mavros_msgs.msg.RCIn

class RCOut(bag_records.records.RecordBase):
    """Record for mavros_msgs/RCOut types
    """
    def __init__(self, has_msg_time=True):
        """Constructor

        Arguments:
            has_msg_time: whether this message has a message time field.
                Defaults True

        Returns:
            class instance
        """
        super(RCOut, self).__init__(has_msg_time)
        self._fields = {
            'bag_time': [],
            'msg_time': [],
            'channels': [],
            }
        self._type = mavros_msgs.msg.RCOut

class AttitudeTarget(bag_records.records.RecordBase):
    """Record for mavros_msgs/AttitudeTarget types
    """
    def __init__(self, has_msg_time=True):
        """Constructor

        Arguments:
            has_msg_time: whether this message has a message time field.
                Defaults True

        Returns:
            class instance
        """
        super(AttitudeTarget, self).__init__(has_msg_time)
        self._fields = {
            'bag_time': [],
            'msg_time': [],
            'type_mask': [],
            'orientation':
                bag_records.geometry_records.Quaternion(has_msg_time=True),
            'body_rate':
                bag_records.geometry_records.Vector3(has_msg_time=True),
            'thrust': [],
            }
        self._type = mavros_msgs.msg.AttitudeTarget

class GlobalPositionTarget(bag_records.records.RecordBase):
    """Record for mavros_msgs/GlobalPositionTarget types
    """
    def __init__(self, has_msg_time=True):
        """Constructor

        Arguments:
            has_msg_time: whether this message has a message time field.
                Defaults True

        Returns:
            class instance
        """
        super(GlobalPositionTarget, self).__init__(has_msg_time)
        self._fields = {
            'bag_time': [],
            'msg_time': [],
            'coordinate_frame': [],
            'type_mask': [],
            'latitude': [],
            'longitude': [],
            'altitude': [],
            'velocity':
                bag_records.geometry_records.Vector3(has_msg_time=True),
            'acceleration_or_force':
                bag_records.geometry_records.Vector3(has_msg_time=True),
            'yaw': [],
            'yaw_rate': [],
            }
        self._type = mavros_msgs.msg.GlobalPositionTarget

class State(bag_records.records.RecordBase):
    """Record for mavros_msgs/State types
    """
    def __init__(self, has_msg_time=True):
        """Constructor

        Arguments:
            has_msg_time: whether this message has a message time field.
                Defaults True

        Returns:
            class instance
        """
        super(State, self).__init__(has_msg_time)
        self._fields = {
            'bag_time': [],
            'msg_time': [],
            'connected': [],
            'armed': [],
            'guided': [],
            'mode': [],
            'system_status': [],
            }
        self._type = mavros_msgs.msg.State

class VFR_HUD(bag_records.records.RecordBase):
    """Record for mavros_msgs/VFR_HUD types
    """
    def __init__(self, has_msg_time=True):
        """Constructor

        Arguments:
            has_msg_time: whether this message has a message time field.
                Defaults True

        Returns:
            class instance
        """
        super(VFR_HUD, self).__init__(has_msg_time)
        self._fields = {
            'bag_time': [],
            'msg_time': [],
            'airspeed': [],
            'groundspeed': [],
            'heading': [],
            'throttle': [],
            'altitude': [],
            'climb': [],
            }
        self._type = mavros_msgs.msg.VFR_HUD

