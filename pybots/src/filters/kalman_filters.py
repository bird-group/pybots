import numpy
import pdb

class KalmanFilter(object):
	def __init__(self):
            self.P = numpy.zeros((0))
            self.Q = numpy.zeros((0))
            self.x = numpy.zeros((0))
            self.z = numpy.zeros((0))

class LKF(KalmanFilter):
	# state					- the initial state vector
	# state_transition		- the "A matrix" state transition as a
	#							function of the current state
	# input_model			- the "B matrix" state transition as a
	#							function of the system input
	# sensor_model			- the "C matrix" system output as a
	#							function of the current state
	# state_covariance		- the initial uncertainty in the state
	# process_noise			- the uncertainty in the system dynamics
	# sensor_noise			- the noise in the measurements
	#
	# Note that this object expects that you've already expressed
	# your system in discrete time -- that is that A describes
	# x_k|k-1 NOT xdot. Also that you scaled process noise properly
	# for the time step size. Also, the only matrix that shouldn't be
	# a tuple of numpy arrays is the state covariance
	def __init__(self,
		state=0,
		state_transition=(numpy.ones((1))),
		input_model=(numpy.zeros((1))),
		sensor_model=(numpy.ones((1))),
		state_covariance=1,
		process_noise=(numpy.ones((1))),
		sensor_noise=(numpy.ones((1))),
		):

		# put all of the system definitions into the object
		self.state = state
		self.state_transition = state_transition
		self.input_model = input_model
		self.n_input_models = len(input_model)
		self.sensor_model = sensor_model
		self.state_covariance = state_covariance
		self.process_noise = process_noise
		self.sensor_noise = sensor_noise
		self.n_sensor_models = len(sensor_model)

		self.innovation_covariance = []
		for i in self.sensor_model:
			self.innovation_covariance.append(
                            numpy.identity((i.shape[0])))

	# state update function, this will take an input and compute the new
	# state estimate, and state covariance
	#
	# stateUpdate(system_input)
	# system_input		- (mx1) numpy array of input to the system
	# system_index		- indicates which system model is being updated
	#
	# updates occur internally so(no returns)
	def state_update(self, system_input, system_index=0):
		i = system_index

		# compute the evolution of the state
		self.state = numpy.dot(self.state_transition[i], self.state) + \
			numpy.dot(self.input_model[i], system_input)

		# compute the new covariance matrix
		self.state_covariance = numpy.dot(self.state_transition[i], \
			numpy.dot(self.state_covariance, self.state_transition[i])) + \
			self.process_noise[i]

	# measurement update function, this will take a measurement and
	# compute the new state estimate and state covariance
	#
	# measurementUpdate(measurement)
	# measurement 		- (px1) numpy array of measurements
	# system_index		- indicates which measurement model to use
	def measurement_update(self, measurement, system_index=0):
		i = system_index

		# compute the innovation
		innovation = measurement - numpy.dot(self.sensor_model[i], self.state)

		# innovation covariance
		self.innovation_covariance[i] = numpy.dot(self.sensor_model[i],
			numpy.dot(self.state_covariance, self.sensor_model[i].transpose())) \
			+ self.sensor_noise[i]

		# Kalman Gain
		kalman_gain = numpy.linalg.solve(self.innovation_covariance[i],
			numpy.dot(self.state_covariance, self.sensor_model[i].transpose()).transpose()).transpose()

		# new state
		self.state += numpy.dot(kalman_gain, innovation)

		# new covariance
		self.state_covariance = numpy.dot(numpy.identity(self.state_covariance.shape[0]) \
			- numpy.dot(kalman_gain, self.sensor_model[i]), self.state_covariance)


class ContinuousTimeKalmanFilter(KalmanFilter):
    """Implements a kalman filter that is defined in countinous time

    Xdot = AX + BU
    Z = CX + DU
    """
    def __init__(self,
        state,
        state_transition,
        input_model,
        sensor_model,
        state_covariance,
        process_noise,
        sensor_noise):
        """Constructor

        Arguments:
            state: initial state vector
            state_transition: Xdot = state_transition * state + ...
            input_model: tuple of input models (to enable several inputs to
                be evaluated) Xdot = ... + input_model * input
            sensor_model: tuple of sensor models (to enable several
                measurements to be incoporated) Z = sensor_model * X
            state_covariance: initial state covariance
            process_noise: process noise covariance does _NOT_ include the
                dt^2 term
            sensor_noise: sensor noise covariance

        Returns:
            class instance
        """

        # put all of the system definitions into the object
        self._state = state
        self._state_transition = state_transition
        self._input_model = input_model
        self._n_input_models = len(input_model)
        self._sensor_model = sensor_model
        self._state_covariance = state_covariance
        self._process_noise = process_noise
        self._sensor_noise = sensor_noise
        self._n_sensor_models = len(sensor_model)

        self._innovation_covariance = []
        for i in self._sensor_model:
            self._innovation_covariance.append(numpy.identity((i.shape[0])))

    @property
    def state(self):
        """Getter for the state
        """
        return self._state

    def state_update(self, system_input, dt, system_index=0):
        """Update the state of the system

        Uses forward euler

        Arguments:
            system_input: numpy (m,1) giving the input
            dt: time step size
            system_index: index of the system we're using to update

        Returns:
            no returns
        """
        i = system_index

        # compute the evolution of the state
        self._state += (
            numpy.dot(self._state_transition[i], self._state) +
            numpy.dot(self._input_model[i], system_input)) * dt

        # compute the new covariance matrix
        self._state_covariance += (
            numpy.dot(self._state_transition[i],
                numpy.dot(self._state_covariance, self._state_transition[i])) +
            self._process_noise[i]) * numpy.power(dt, 2.0)

    def measurement_update(self, measurement, system_index=0):
        """Update the state given a measurement

        Arguments:
            measurement: (p,1) numpy vector giving measurement
            system_index: indicats which measurement model to use

        Returns:
            no returns
        """
        i = system_index

        #z = measurement
        #H = self._sensor_model[i]
        #P = self._state_covariance
        #R = self._sensor_noise[i]
        #x = self._state

        # compute the innovation
        innovation = measurement - numpy.dot(
            self._sensor_model[i], self._state)

        # innovation covariance
        self._innovation_covariance[i] = (
            numpy.dot(
                self._sensor_model[i],
                numpy.dot(
                    self._state_covariance,
                    self._sensor_model[i].transpose())) +
            self._sensor_noise[i])

        # Kalman Gain
        kalman_gain = numpy.linalg.solve(
            self._innovation_covariance[i].T,
            self._state_covariance.dot(self._sensor_model[i].T).T).T

        # new state
        self._state += numpy.dot(kalman_gain, innovation)

        # new covariance
        self._state_covariance = numpy.dot(
            (
                numpy.identity(self._state_covariance.shape[0]) -
                numpy.dot(kalman_gain, self._sensor_model[i])),
            self._state_covariance)
