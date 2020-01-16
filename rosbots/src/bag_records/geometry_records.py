import pdb

import numpy

import geometry.quaternion

import bag_records.records

import copy

class Accel(bag_records.records.RecordBase):
    """Record for geometry_msgs/Accel types
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
        super(Accel, self).__init__(has_msg_time)
        self._fields = {
            'bag_time': [],
            'msg_time': [],
            'linear': Vector3(has_msg_time=has_msg_time),
            'angular': Vector3(has_msg_time=has_msg_time),
            }

class AccelStamped(bag_records.records.RecordBase):
    """Record for geometry_msgs/AccelStamped types
    """
    def __init__(self, has_msg_time=True):
        """Constructor

        Arguments:
            no arguments

        Returns:
            class instance
        """
        super(AccelStamped, self).__init__(has_msg_time)
        self._fields = {
            'bag_time': [],
            'msg_time': [],
            'accel': Accel(has_msg_time=True)
            }

class Pose(bag_records.records.RecordBase):
    """Record for geometry_msgs/Pose types
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
        super(Pose, self).__init__(has_msg_time)
        self._fields = {
            'bag_time': [],
            'msg_time': [],
            'position': Point(has_msg_time=has_msg_time),
            'orientation': Quaternion(has_msg_time=has_msg_time),
            }

class PoseStamped(bag_records.records.RecordBase):
    """Record for geometry_msgs/PoseStamped types
    """
    def __init__(self, has_msg_time=True):
        """Constructor

        Arguments:
            no arguments

        Returns:
            class instance
        """
        super(PoseStamped, self).__init__(has_msg_time)
        self._fields = {
            'bag_time': [],
            'msg_time': [],
            'pose': Pose(has_msg_time=True)
            }

class Quaternion(bag_records.records.RecordBase):
    """Record for geometry_msgs/Quaternion types
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
        super(Quaternion, self).__init__(has_msg_time)
        self._fields = {
            'bag_time': [],
            'msg_time': [],
            'x': [],
            'y': [],
            'z': [],
            'w': [],
            }

    @property
    def quaternion(self):
        """Get a bunch of quaternions from the record data

        Arguments:
            no arguments

        Returns:
            q: list of quaternions corresponding to the quaternion stored here
        """
        q = [
            geometry.quaternion.Quaternion(numpy.array([w, x, y, z])) for
            w, x, y, z in zip(
                self._fields['w'],
                self._fields['x'],
                self._fields['y'],
                self._fields['z'])]
        return q

class QuaternionStamped(bag_records.records.RecordBase):
    """Record for geometry_msgs/QuaternionStamped types
    """
    def __init__(self, has_msg_time=True):
        """Constructor

        Arguments:
            no arguments

        Returns:
            class instance
        """
        super(QuaternionStamped, self).__init__(has_msg_time)
        self._fields = {
            'bag_time': [],
            'msg_time': [],
            'pose': Quaternion(has_msg_time=True)
            }

    @property
    def quaternion(self):
        """Get a bunch of quaternions from the record data

        Arguments:
            no arguments

        Returns:
            q: list of quaternions corresponding to the quaternion stored here
        """
        return self._fields['pose'].quaternion

class Twist(Accel):
    """Record for geometry_msgs/Twist types
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
        super(Twist, self).__init__(has_msg_time)

class TwistStamped(bag_records.records.RecordBase):
    """Record for geometry_msgs/AccelStamped types
    """
    def __init__(self, has_msg_time=True):
        super(TwistStamped, self).__init__(has_msg_time)
        self._fields = {
            'bag_time': [],
            'msg_time': [],
            'twist': Twist(has_msg_time=True)
            }

class TwistWithCovariance(bag_records.records.RecordBase):
    """Record for geometry_msgs/TwistWithCovariance types
    """
    def __init__(self, has_msg_time=True):
        super(TwistWithCovariance, self).__init__(has_msg_time)
        self._fields = {
            'bag_time': [],
            'msg_time': [],
            'twist': Twist(has_msg_time=True),
            'covariance': [],
            }

class TwistWithCovarianceStamped(bag_records.records.RecordBase):
    """Record for geometry_msgs/TwistWithCovarianceStamped types
    """
    def __init__(self, has_msg_time=True):
        super(TwistWithCovarianceStamped, self).__init__(has_msg_time)
        self._fields = {
            'bag_time': [],
            'msg_time': [],
            'twist': TwistWithCovariance(has_msg_time=True)
            }

class Vector3(bag_records.records.RecordBase):
    """Record for geometry_msgs/Vector3 types
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
        super(Vector3, self).__init__(has_msg_time)
        self._fields = {
            'bag_time': [],
            'msg_time': [],
            'x': [],
            'y': [],
            'z': []
            }

    @property
    def to_numpy(self, has_msg_time=True):
        """Return an nx3 numpy array of this data

        Arguments:
            no arguments

        Returns:
            numpy_array: numpy nx3 array of vector3 data
        """
        numpy_array = numpy.vstack((
            self._fields['x'],
            self._fields['y'],
            self._fields['z']))
        return numpy_array.T

class Vector3Stamped(bag_records.records.RecordBase):
    """Record for geometry_msgs/Vector3Stamped types
    """
    def __init__(self, has_msg_time=True):
        """Constructor

        Arguments:
            no arguments

        Returns:
            class instance
        """
        super(Vector3Stamped, self).__init__(has_msg_time)
        self._fields = {
            'bag_time': [],
            'msg_time': [],
            'vector': Vector3(has_msg_time=True),
            }

    @property
    def to_numpy(self, has_msg_time=True):
        """Return an nx3 numpy array of this data

        Arguments:
            no arguments

        Returns:
            numpy_array: numpy nx3 array of vector3 data
        """
        return self._fields['vector'].to_numpy

class Point(Vector3):
    """Record for geometry_msgs/Point types
    """
    def __init__(self, has_msg_time=False):
        """Constructor
        """
        super(Point, self).__init__(has_msg_time)

class PointStamped(bag_records.records.RecordBase):
    """Record for geometry_msgs/PointStamped types
    """
    def __init__(self, has_msg_time=True):
        """Constructor

        Arguments:
            no arguments

        Returns:
            class instance
        """
        super(PointStamped, self).__init__(has_msg_time)
        self._fields = {
            'bag_time': [],
            'msg_time': [],
            'point': Point(has_msg_time=True),
            }

    @property
    def to_numpy(self, has_msg_time=True):
        """Return an nx3 numpy array of this data

        Arguments:
            no arguments

        Returns:
            numpy_array: numpy nx3 array of vector3 data
        """
        return self._fields['point'].to_numpy

class Polygon(bag_records.records.RecordBase):
    """Record for geometry_msgs/Polygon types
    """
    def __init__(self, has_msg_time=False):
        """constructor
        """
        super(Polygon, self).__init__(has_msg_time)
        self._fields = {
            'bag_time': [],
            'msg_time': [],
            'points': [],
            }
        self._array_types = {
            'points': Point,
            }

class PolygonStamped(bag_records.records.RecordBase):
    """Record for geometry_msgs/PolygonStamped types
    """
    def __init__(self):
        """constructor
        """
        super(PolygonStamped, self).__init__(has_msg_time=True)
        self._fields = {
            'bag_time': [],
            'msg_time': [],
            'polygon': Polygon(has_msg_time=True),
            }

