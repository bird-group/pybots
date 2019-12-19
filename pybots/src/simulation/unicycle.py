import numpy

import simulation.dynamic_system

class Unicycle(simulation.dynamic_system.DynamicSystem):
    """Implements the dynamics to simulate a unicycle"""
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
                psi: heading
            dt: time step size
            output_fcn: output function, takes X and returns Y
            input_fcn: a function to determine input from the state
                for example a controller

        Returns:
            object instance
        """
        super(Unicycle, self).__init__(
            X0, dt, self._dynamics, output_fcn, input_fcn)
        self._U = numpy.zeros((2,))

    def _dynamics(self, X, U):
        """Compute the system dynamics

        Arguments:
            X: state
                x: x position
                y: y position
                psi: heading
            U: input
                psi_dot: turn rate
                s: speed
        """
        xdot = U[1] * numpy.cos(X[2])
        ydot = U[1] * numpy.sin(X[2])
        psi_dot = U[0]
        X_dot = numpy.array([xdot, ydot, psi_dot])
        return X_dot

    @property
    def X(self):
        """Get the position
        """
        return self._X[0:2]

    @property
    def psi(self):
        """Get the heading
        """
        return self._X[2]

    @property
    def velocity(self):
        """Get the velocity
        """
        V = numpy.array([
            numpy.cos(self.psi) * self.input[1],
            numpy.sin(self.psi) * self.input[1]
            ])
        return V

class DynamicUnicycle(simulation.dynamic_system.DynamicSystem):
    """Implements the dynamics to simulate a unicycle"""
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
                v: speed
                psi: heading
            dt: time step size
            output_fcn: output function, takes X and returns Y
            input_fcn: a function to determine input from the state
                for example a controller

        Returns:
            object instance
        """
        super(DynamicUnicycle, self).__init__(
            X0, dt, self._dynamics, output_fcn, input_fcn)
        self._U = numpy.zeros((2,))

    def _dynamics(self, X, U):
        """Compute the system dynamics

        Arguments:
            X: state
                x: x position
                y: y position
                v: speed
                psi: heading
            U: input
                a: acceleration in direction of travel
                psi_dot: turn rate
        """
        xdot = X[2] * numpy.cos(X[3])
        ydot = X[2] * numpy.sin(X[3])
        v_dot = U[0]
        psi_dot = U[1]
        X_dot = numpy.array([xdot, ydot, v_dot, psi_dot])
        return X_dot

    @property
    def position(self):
        """Get the position
        """
        return self._X[0:2]

    @property
    def psi(self):
        """Get the heading
        """
        return self._X[3]

    @property
    def speed(self):
        """Get the speed
        """
        return self.X[2]

    @property
    def velocity(self):
        """Get the velocity
        """
        V = numpy.array([
            numpy.cos(self.psi) * self._X[3],
            numpy.sin(self.psi) * self._X[3]
            ])
        return V

class ThreeDUnicycle(simulation.dynamic_system.DynamicSystem):
    """Implements the dynamics to simulate a unicycle in 3-d space
    """
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
                z: z position
                psi: heading
            dt: time step size
            output_fcn: output function, takes X and returns Y
            input_fcn: a function to determine input from the state
                for example a controller

        Returns:
            object instance
        """
        super(ThreeDUnicycle, self).__init__(
            X0, dt, self._dynamics, output_fcn, input_fcn)
        self._U = numpy.zeros((3,))

    def _dynamics(self, X, U):
        """Compute the system dynamics

        Arguments:
            X: state
                x: x position
                y: y position
                z: z position
                psi: heading
            U: input
                psi_dot: turn rate
                s: speed
                w: vertical speed
        """
        xdot = U[1] * numpy.cos(X[3])
        ydot = U[1] * numpy.sin(X[3])
        zdot = U[2]
        psi_dot = U[0]
        X_dot = numpy.array([xdot, ydot, zdot, psi_dot])
        return X_dot

    @property
    def X(self):
        """Get the position
        """
        return self._X[0:3]

    @property
    def psi(self):
        """Get the heading
        """
        return self._X[3]

    @property
    def velocity(self):
        """Get the velocity
        """
        V = numpy.array([
            numpy.cos(self.psi) * self.input[1],
            numpy.sin(self.psi) * self.input[1],
            self.input[2]])
        return V
