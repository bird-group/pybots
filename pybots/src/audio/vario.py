from audio.sounds import beep
import numpy as np
import time

class VarioTone(object):
    """ A class to make vario sounds
    """
    def __init__(self, max_val=5.0):
        """ Constructor
Arguments:
            max_val: optional (defaults to 5), the saturation vario reading in
                meters per second

        Returns:
            class instance
        """
        super(VarioTone, self).__init__()

        self._max = max_val

        self._is_running = False
        self._val = 0.0
        self._last_val = 0.0
        self._beep_time = time.time()
        self._beep_duration = 0.0
        self._beep_dt = 1.0
        self._thread = None

    def start_vario(self):
        """ Start the vario running
        """
        self._is_running = True
        while self._is_running is True:
            self._service()
            dt = min(0.1, self._beep_dt/2.0)
            time.sleep(dt)

    def stop_vario(self):
        """ stop the vario
        """
        self._is_running = False

    def _service(self):
        """ make the beeps
        """
        f = 260.0 + (3000.0 - 260.0)/self._max*(
            np.clip(self._val, 0.0, self._max))

        dt = 0.3 + (0.03 - 0.3)/self._max*(
            np.clip(self._val, 0.0, self._max))

        spacing = 1.0 + (0.1 - 1.0)/self._max*(
            np.clip(self._val, 0.0, self._max))

        if (time.time() - self._beep_time > self._beep_duration or
            abs(self._val - self._last_val) > 0.0):
            if self._val > 0.001 :
                self._beep_duration = dt*spacing + dt
                self._beep_dt = dt
                beep(dt, f)

