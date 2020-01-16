import pdb

import copy

import numpy

import geometry.spline_model

import std_msgs.msg
import robots_common.msg
import geometry_msgs.msg

import geometry.spline_model

import bag_records.records
import bag_records.geometry_records

class SplineModel(bag_records.records.RecordBase):
    """Record for robots_common/SplineModel types
    """
    def __init__(self, has_msg_time=True):
        """Constructor

        Arguments:
            no arguments

        Returns:
            class instance
        """
        super(SplineModel, self).__init__(has_msg_time)
        self._fields = {
            'bag_time': [],
            'msg_time': [],
            'knots': [],
            'coefficients': [],
            'covariance': [],
            'order': []
            }
        self._type = robots_common.msg.SplineModel

    def to_spline(self, idx=None):
        """Create a spline model from a record

        Arguments:
            idx: index to convert, or iterable of indices. If not specified all
                entries will be converted

        Returns:
            splines: SplineModel object or a list of them matching the
                specified indices
        """
        if isinstance(idx, int):
            n_coefficients = len(self._fields['coefficients'][idx])
            order = self._fields['order'][idx]
            knots = self._fields['knots'][idx]
            P = numpy.reshape(
                self._fields['covariance'][idx],
                (n_coefficients, n_coefficients))
            bknots = (knots[:order], knots[-order:])
            spline = geometry.spline_model.SplineModel(
                knots[order:-order],
                order,
                self._fields['coefficients'][idx],
                P=P,
                boundary_knots=bknots)
            return spline

        if idx is None:
            idx = range(len(self))
        splines = []
        for i in idx:
            splines.append(self.to_spline(i))
        return splines

class Profile(bag_records.records.RecordBase):
    """Record for robots_common/Profile types
    """
    def __init__(self, has_msg_time=True):
        """Constructor

        Arguments:
            no arguments

        Returns:
            class instance
        """
        super(Profile, self).__init__(has_msg_time)
        self._fields = {
            'bag_time': [],
            'msg_time': [],
            'altitude': [],
            'mean': [],
            'sigma': [],
            }
        self._type = robots_common.msg.Profile

class Float32Stamped(bag_records.records.RecordBase):
    """Record for robots_common/Float32Stamped types
    """
    def __init__(self, has_msg_time=True):
        """Constructor

        Arguments:
            no arguments

        Returns:
            class instance
        """
        super(Float32Stamped, self).__init__(has_msg_time)
        self._fields = {
            'bag_time': [],
            'msg_time': [],
            'data': [],
            }
        self._type = robots_common.msg.Float32Stamped

class Float64Stamped(bag_records.records.RecordBase):
    """Record for robots_common/Float64Stamped types
    """
    def __init__(self, has_msg_time=True):
        """Constructor

        Arguments:
            has_msg_time: optional bool indicating if this record should have
                a message time. Typically True

        Returns:
            class instance
        """
        super(Float64Stamped, self).__init__()
        self._fields = {'data': [], 'bag_time': [], 'msg_time': []}
        self._type = robots_common.msg.Float64Stamped
        self._has_msg_time = has_msg_time

class UInt8ArrayStamped(bag_records.records.RecordBase):
    """Record for robots_common/UInt8ArrayStamped types
    """
    def __init__(self, has_msg_time=True):
        """Constructor

        Arguments:
            has_msg_time: optional bool indicating if this record should have
                a message time. Typically True

        Returns:
            class instance
        """
        super(UInt8ArrayStamped, self).__init__()
        self._fields = {'data': [], 'bag_time': [], 'msg_time': []}
        self._array_types = {
            'data': []
            }
        self._type = robots_common.msg.UInt8ArrayStamped
        self._has_msg_time = has_msg_time

class Waypoint(bag_records.records.RecordBase):
    """Record for rosbots/Waypoint message
    """
    def __init__(self):
        super(Waypoint, self).__init__(has_msg_time=True, interpolate=False)
        self._fields = {
            'bag_time': [],
            'msg_time': [],
            'number': [],
            'next_number': [],
            'lla': bag_records.geometry_records.Vector3(has_msg_time=True),
            'orbit_time': [],
            'description': [],
            }
        self._type = robots_common.msg.SplineModel

class WaypointVector(bag_records.records.RecordBase):
    """Record for rosbots/WaypointVector message
    """
    def __init__(self):
        super(WaypointVector, self).__init__(
            has_msg_time=True, interpolate=False)
        self._fields = {
            'bag_time': [],
            'msg_time': [],
            'waypoints': [],
            }
        self._array_types = {'waypoints': Waypoint}

