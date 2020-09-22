import pdb

import numpy

class PotentialField(object):
    def __init__(self, primitives):
        self._potentials = primitives

    def potential(self, X):
        return numpy.sum([p.potential(X) for p in self._potentials])

    def u(self, X):
        return numpy.sum([p.u(X) for p in self._potentials])

    def v(self, X):
        return numpy.sum([p.v(X) for p in self._potentials])

    def V(self, X):
        return numpy.sum([p.V(X) for p in self._potentials], axis=0)

class PotentialPrimitive(object):
    """
    """
    def __init__(self):
        return

    def potential(self, X):
        """Get the potential value at a point
        """
        return self._potential_function(X)

    def u(self, X):
        """Get x velocity at a point
        """
        return self._potential_gradient(X)[0]

    def v(self, X):
        """Get y velocity at a point
        """
        return self._potential_gradient(X)[1]

    def V(self, X):
        return self._potential_gradient(X)

class UniformFlow(PotentialPrimitive):
    def __init__(self, u, v):
        """
        Arguments:
            u: x velocity
            v: y velocity
        """
        self._a = u
        self._b = v
        self._c = numpy.array([self._a, self._b])

    def _potential_function(self, X):
        return self._c.dot(X)

    def _potential_gradient(self, X):
        return self._c

class Source(PotentialPrimitive):
    def __init__(self, strength, X0):
        """
        Arguments:
            strength: source strength
            X0: source position
        """
        self._X0 = X0
        self._m = strength

    def _potential_function(self, X):
        """
        """
        r = numpy.linalg.norm(X - self._X0)
        return self._m * numpy.log(r)

    def _potential_gradient(self, X):
        r2 = numpy.power(numpy.linalg.norm(X - self._X0), 2.0)
        dphi_dX = self._m * (X - self._X0) / r2
        return dphi_dX

class Vortex(PotentialPrimitive):
    def __init__(self, strength, X0):
        """
        Arguments:
            strength: vortex strength
            X0: vortex position
        """
        self._X0 = X0
        self._gamma = strength

    def _potential_function(self, X):
        """
        """
        Xlocal = X - self._X0
        theta = numpy.arctan(Xlocal[1] / Xlocal[0])
        return self._gamma / numpy.pi / 2.0 * theta

    def _potential_gradient(self, X):
        r2 = numpy.power(numpy.linalg.norm(X - self._X0), 2.0)
        Xlocal = X - self._X0
        dphi_dX = (
            numpy.stack([-Xlocal[1], Xlocal[0]]) *
            self._gamma / numpy.pi / 2.0 / r2
            )
        return dphi_dX

class Doublet(PotentialPrimitive):
    def __init__(self, strength, X0, direction):
        """
        Arguments:
            strength: doublet strength
            X0: doublet position
            direction: vector giving the doublet's direction
        """
        self._X0 = X0
        self._mu = strength
        self._direction = numpy.arctan2(direction[1], direction[0])

    def _potential_function(self, X):
        Xlocal = X - self._X0
        r = numpy.linalg.norm(Xlocal)
        theta = numpy.arctan(Xlocal[1] / Xlocal[0])
        return (
            self._mu / numpy.pi / 2.0 * numpy.cos(theta - self._direction) / r)

    def _potential_gradient(self, X):
        Xlocal = X - self._X0
        r2 = numpy.power(numpy.linalg.norm(Xlocal), 2.0)

        scale = -self._mu / numpy.pi / 2.0 / r2 / r2
        squared_diff = numpy.diff(numpy.power(Xlocal, 2.0))[0]
        u = scale * (
            numpy.cos(self._direction) * squared_diff -
            2.0 * numpy.prod(Xlocal) * numpy.sin(self._direction))
        v = scale * (
            numpy.sin(self._direction) * squared_diff -
            2.0 * numpy.prod(Xlocal) * numpy.cos(self._direction))
        return numpy.array([u,v])

        dtheta = -(numpy.arctan(Xlocal[1] / Xlocal[0]) - self._direction)
        scale = self._mu / numpy.pi / 2.0 / numpy.power(r2, 1.5)
        u = -scale * (
            Xlocal[0] * numpy.cos(dtheta) + Xlocal[1] * numpy.sin(dtheta))
        v = scale * (
            Xlocal[0] * numpy.sin(dtheta) - Xlocal[1] * numpy.cos(dtheta))
        return numpy.array([u,v])
