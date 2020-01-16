import numpy

import simulation.dynamic_system

class DynamicParticle(simulation.dynamic_system.DynamicSystem):
    """Implements the dynamics to simulate a particle with dynamics"""
    def __init__(
        self,
        X0,
        dt,
        output_fcn=None,
        input_fcn=None):
        """Constructor

        Arguments:
            X0: initial state
                x: x position
                y: y position
                vx: x velocity
                vy: y velocity
            dt: time step size
            output_fcn: output function, takes X and returns Y
            input_fcn: a function to determine input from the state
                for example a controller

        Returns:
            object instance
        """
        super(DynamicParticle, self).__init__(
            X0, dt, self._dynamics, output_fcn, input_fcn)
        self._U = numpy.zeros((2,))

    def _dynamics(self, X, U):
        """Compute the system dynamics

        Arguments:
            X: state
                x: x position
                y: y position
                vx: x velocity
                vy: y velocity
            U: input
                ax: x acceleration
                ay: y acceleration
        """
        xdot = X[2]
        ydot = X[3]
        vxdot = U[0]
        vydot = U[1]
        X_dot = numpy.array([xdot, ydot, vxdot, vydot])
        return X_dot

    @property
    def position(self):
        """Get the position
        """
        return self._X[0:2]

    @property
    def velocity(self):
        """Get the heading
        """
        return self._X[2:4]

