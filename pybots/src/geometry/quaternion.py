"""
quaternion operations, written at some point and finished up on 25 Sep 2014

initialize using quaternion(x), defaults to a unit quaternion
representing a 0 rotation
John Bird
"""
import pdb
import numpy
import copy as copy

def from_euler(euler):
    """ Set the quaternion from an euler array

    Arguments:
        euler: 3, numpy array of phi,theta,psi

    Returns:
        no returns
    """
    euler = numpy.array(euler)

    cos_f_phi = numpy.cos(euler[0] / 2.0)
    sin_f_phi = numpy.sin(euler[0] / 2.0)
    cos_f_theta = numpy.cos(euler[1] / 2.0)
    sin_f_theta = numpy.sin(euler[1] / 2.0)
    cos_f_psi = numpy.cos(euler[2] / 2.0)
    sin_f_psi = numpy.sin(euler[2] / 2.0)

    x = numpy.zeros((4,))

    # equations for computing the quaternion components from euler angles
    x[0] = (cos_f_phi * cos_f_theta * cos_f_psi +
            sin_f_phi * sin_f_theta * sin_f_psi)
    x[1] = (sin_f_phi * cos_f_theta * cos_f_psi -
            cos_f_phi * sin_f_theta * sin_f_psi)
    x[2] = (cos_f_phi * sin_f_theta * cos_f_psi +
            sin_f_phi * cos_f_theta * sin_f_psi)
    x[3] = (cos_f_phi * cos_f_theta * sin_f_psi -
            sin_f_phi * sin_f_theta * cos_f_psi)

    return UnitQuaternion(x=x)

def from_vector(vector, unit=False):
    """Construct a quaternion from a 3 element vector
    """
    x = numpy.hstack((numpy.zeros((1,)), numpy.squeeze(vector)))
    if unit:
        return UnitQuaternion(x)
    else:
        return Quaternion(x)

class Quaternion(object):
    """ quaternion class
    """
    def __init__(self, x=numpy.array([1.0, 0.0, 0.0, 0.0])):
        """ constructor

        Arguments:
            x: components, defaults to [1,0,0,0] (no rotation)

        Returns:
            the object
        """
        self.x = numpy.array(x)

    def from_euler(self, euler):
        """ Set the quaternion from an euler array

        Arguments:
            euler: 3, numpy array of phi,theta,psi

        Returns:
            no returns
        """
        euler = numpy.array(euler)
        phi = euler[0]
        theta = euler[1]
        psi = euler[2]

        x = numpy.zeros((4))

        cos_f_phi = numpy.cos(euler[0] / 2.0)
        sin_f_phi = sin(euler[0] / 2.0)
        cos_f_theta = numpy.cos(euler[1] / 2.0)
        sin_f_theta = sin(euler[1] / 2.0)
        cos_f_psi = numpy.cos(euler[2] / 2.0)
        sin_f_psi = sin(euler[2] / 2.0)

        # equations for computing the quaternion components from euler angles
        x[0] = (cos_f_phi * cos_f_theta * cos_f_psi +
                sin_f_phi * sin_f_theta * sin_f_psi)
        x[1] = (sin_f_phi * cos_f_theta * cos_f_psi -
                cos_f_phi * sin_f_theta * sin_f_psi)
        x[2] = (cos_f_phi * sin_f_theta * cos_f_psi +
                sin_f_phi * cos_f_theta * sin_f_psi)
        x[3] = (cos_f_phi * cos_f_theta * sin_f_psi -
                sin_f_phi * sin_f_theta * cos_f_psi)

        self.x = x

    def from_axis_and_rotation(self, axis, rotation):
        """Set the quaternion from a rotation axis and rotation magnitude

        Arguments:
            axis: unit vector in direction of axis, numpy (3,) array
            rotation: magnitude of rotation (rad)

        Returns:
            no returns
        """
        x = numpy.zeros((4,))

        # from http://paulbourke.net/geometry/rotate/
        x[0] = numpy.cos(rotation / 2.0)
        x[1:] = numpy.sin(rotation / 2.0) * axis
        self.x = x

    def __div__(self, other):
        """ Division operator for quaternions

        Arguments:
            other: the quaternion we're going to divide by this one

        Returns:
            other/this
        """
        return other * self.inverse()

    def __truediv__(self, other):
        """ Division operator for quaternions

        Arguments:
            other: the quaternion we're going to divide by this one

        Returns:
            other/this
        """
        return self.__div__(other)

    def _fast_cross(self, x1, x2):
        """A fast cross product for three element vectors

        Numpy is really slow for repeated calls to cross product for small
        matrices
        """
        v = numpy.array([
            x1[1] * x2[2] - x1[2] * x2[1],
            x1[2] * x2[0] - x1[0] * x2[2],
            x1[0] * x2[1] - x1[1] * x2[0]
        ])
        return v

    def __mul__(self, other):
        """ multiplication operator for quaternions

        Arguments:
            other: the quaternion we're going to multiply by

        Returns:
            product
        """
        x1 = self.x
        x2 = other.x
        x = numpy.zeros((4,))

        x[0] = x1[0] * x2[0] - numpy.dot(x1[1:4], x2[1:4])
        x[1:4] = (
            x1[0] * x2[1:4] + x2[0] * x1[1:4] +
            self._fast_cross(x1[1:4], x2[1:4]))
        return Quaternion(x)

    def inverse(self):
        """ defines inverse for quaternion

        Arguments:
            no arguments

        Returns:
            no returns
        """
        inv = numpy.array([1.0, -1.0, -1.0, -1.0]) / self.norm()
        return Quaternion(self.x * inv)

    def conjugate(self):
        """ defines conjugate of quaternion

        Arguments:
            no arguments

        Returns:
            q_star: the conjugate of this quaternion
        """
        q_star = Quaternion([self.x[0], -self.x[1], -self.x[2], -self.x[3]])
        return q_star

    def norm(self):
        """ Defines norm of quaternion

        Arguments:
            no arguments

        Returns:
            norm: the quaternion norm
        """
        return numpy.sqrt(numpy.sum(numpy.dot(self.x, self.x)))

    def normalize(self):
        """ Normalize the quaternion to a unit length

        The quaternion that this is called on will be modified such that it
        has a unit norm

        Arguments:
            no arguments

        Returns:
            no returns
        """
        self.x /= self.norm()

    def rot(self, v, flag=True):
        """ Rotate a vector through the rotation represented by the quaternion.

        Arguments:
            v:  3, numpy array or something we can convert to it
            flag: defaults true, indicates which way we're going through the
                quaternion. If the orientation represents an aircraft then True
                indicates going from inertial to body axes

        Returns:
            the rotated vector
        """
        x = numpy.hstack((numpy.zeros((1,)), v))
        q = Quaternion(x)

        if flag:
            qq = self.inverse() * q * self
        else:
            qq = self * q * self.inverse()

        return qq.x[1:4]

    def qdot(self, omega):
        """Compute the quaternion rates from angular rates

        Arguments:
            omega: vector of [P, Q, R]

        Returns:
            qdot: rate of change of the quaternion components
        """
        q_omega = from_vector(omega)
        qdot = (self * q_omega).x / 2.0
        return qdot

    @property
    def phi(self):
        """ Compute the equivalent phi angle for an euler triplet

        Arguments:
            no arguments

        Returns:
            phi: angle in radians
        """
        c23 = 2.0 * (self.x[2] * self.x[3] + self.x[0] * self.x[1])
        c33 = (pow(self.x[0], 2.0) - pow(self.x[1], 2.0) -
               pow(self.x[2], 2.0) + pow(self.x[3], 2.0))
        phi = numpy.arctan2(c23, c33)

        return phi

    @property
    def theta(self):
        """ Compute the equivalent theta angle for an euler triplet

        Arguments:
            no arguments

        Returns:
            theta: angle in radians
        """
        c13 = 2.0 * (self.x[1] * self.x[3] - self.x[0] * self.x[2])
        theta = -numpy.arcsin(c13)

        return theta

    @property
    def psi(self):
        """ Compute the equivalent psi angle for an euler triplet

        Arguments:
            no arguments

        Returns:
            psi: angle in radians
        """
        c12 = 2.0 * (self.x[1] * self.x[2] + self.x[0] * self.x[3])
        c11 = (pow(self.x[0], 2.0) + pow(self.x[1], 2.0) -
               pow(self.x[2], 2.0) - pow(self.x[3], 2.0))
        psi = numpy.arctan2(c12, c11)

        return psi

    @property
    def euler(self):
        """ Compute the equivalent euler triplet

        Arguments:
            no arguments

        Returns:
            euler_array: 3, numpy array of euler angles
        """
        euler_array = numpy.array([self.phi, self.theta, self.psi])

        return euler_array

    @property
    def angle_axis(self):
        """Compute the equivalent angle-axis representation

        From the wikipedia article...

        Arguments:
            no arguments

        Returns:
            angle: angle of rotation in radians
            axis: numpy (3,) vector representing the axis
        """
        axis = self.x[1:] / numpy.linalg.norm(self.x[1:])
        angle = 2.0 * numpy.arctan2(numpy.linalg.norm(self.x[1:]), self.x[0])
        return angle, axis

class UnitQuaternion(Quaternion):
    """ a class for unit quaternions
    """
    def __init__(self, x=numpy.array([1.0, 0.0, 0.0, 0.0])):
        super(UnitQuaternion, self).__init__(x)
        self.normalize()

    def __div__(self, other):
        quotient = super(UnitQuaternion, self).__div__(other)
        if type(self) == type(other):
            quotient.normalize()
        return quotient

    def __truediv__(self, other):
        return self.__div__(other)

    def __mul__(self, other):
        product = super(UnitQuaternion, self).__mul__(other)
        if type(self) == type(other):
            product.normalize()
        return product
