import pdb

import smbus

import numpy

class CastlePacket(object):
    """Right now command pcakets are not implemented
    """
    def __init__(self, device_id, register_address):
        """Construct the packet"""

        self.payload = [0, 0, 0, 0]
        self.device_id = device_id
        self.register_address = register_address

    @property
    def checksum(self):
        """Compute the packet checksum"""
        checksum = 256 - (self.payload[0] << 1)
        for byte in self.payload[1:]:
            checksum -= byte
        return checksum

    @property
    def device_id(self):
        """Get the device ID from the payload
        """
        return self.payload[0]

    @device_id.setter
    def device_id(self, new_id):
        """Set the device ID
        """
        self.payload[0] = new_id

    @property
    def register_address(self):
        """Get the register address from the payload
        """
        return self.payload[1]

    @register_address.setter
    def register_address(self, new_address):
        """Set the register address
        """
        self.payload[1] = new_address

    def check_response(self, response_data):
        """Verify the checksum
        """
        response = numpy.asarray(response_data, dtype=numpy.int8)
        if sum(response) == 0:
            return True
        else:
            return False

    @property
    def i2c_payload(self):
        """Return the i2c payload
        """
        command = self.payload[2:]
        command.append(self.checksum)
        return command

class CastleSerialLinkI2C:
    """I2C driver for the serial link
    """
    _divider = 2042.0
    _scale = {
        'voltage': 20.0,
        'ripple': 4.0,
        'current': 50.0,
        'throttle': 1.0,
        'output_power': 0.2502,
        'rpm': 20416.66,
        'bec_voltage': 4.0,
        'bec_current': 4.0,
        'temperature': 30.0,
        }
    _register = {
        'voltage': 0,
        'ripple': 1,
        'current': 2,
        'throttle': 3,
        'output_power': 4,
        'rpm': 5,
        'bec_voltage': 7,
        'bec_current': 8,
        'temperature': 6,
        }
    def __init__(self, i2c_bus=1, device_id=8):
        self._device_id = numpy.array(device_id, dtype=numpy.uint8)
        self._bus = smbus.SMBus(i2c_bus)

    def _read_register(self, reg_address):
        """ read from a register

        uses the castle serial protocol to read a value from register

        Args:
            reg_address: one byte identifying the register

        Returns:
            reg_data: two byte tuple of data from register

        reg_data will return as None if a problem was encountered in
        reading from the device. Calling function should check for this
        condition
        """
        command_pkt = CastlePacket(self._device_id, register_address=reg_address)
        self._bus.write_i2c_block_data(
            command_pkt.device_id,
            command_pkt.register_address,
            command_pkt.i2c_payload)
        response_data = self._bus.read_i2c_block_data(
            command_pkt.device_id,
            command_pkt.register_address,
            3)
        if 256 - numpy.sum(response_data) != 0:
            return None
        return (response_data[0], response_data[1])

    def _scale_data(self, data, field_id):
        """Scale the data to engineering units

        Arguments
            data: output from _read_register
            field_id: string identifying the field

        Returns:
            X: data in engineering units
        """
        X = (((data[0] << 8) + data[1]) /
            self._divider *
            self._scale[field_id])
        return X

    def read_voltage(self):
        data = self._read_register(self._register['voltage'])
        if data is None:
            return data
        else:
            return self._scale_data(data, 'voltage')

    def read_ripple(self):
        data = self._read_register(self._register['ripple'])
        if data is None:
            return data
        else:
            return self._scale_data(data, 'ripple')

    def read_current(self):
        data = self._read_register(self._register['current'])
        if data is None:
            return data
        else:
            return self._scale_data(data, 'current')

    def read_throttle(self):
        data = self._read_register(self._register['throttle'])
        if data is None:
            return data
        else:
            return self._scale_data(data, 'throttle')

    def read_power(self):
        data = self._read_register(self._register['output_power'])
        if data is None:
            return data
        else:
            return self._scale_data(data, 'output_power')

    def read_rpm(self):
        data = self._read_register(self._register['rpm'])
        if data is None:
            return data
        else:
            return self._scale_data(data, 'rpm')

    def read_temp(self):
        data = self._read_register(self._register['temperature'])
        if data is None:
            return data
        else:
            return self._scale_data(data, 'temperature')
