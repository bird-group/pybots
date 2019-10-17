from struct import pack
from struct import unpack

from types import IntType
from types import FloatType
from types import BooleanType
from types import StringType

from common_packets.avia_packet import AVIaPacket

import pdb

class ParameterPacket(AVIaPacket):
    """ Super class for parameter packets

    All are type 0x00, but sub-types are defined  within the message and broken
    out by the given dict.

    The packet structure is:
        bytes 0:n
            parameter name -- bytes forming a string
        byte n
            0xff, divides name from vaue
        byte n+1
            the parameter type, specified in _type_dict
        byte n+2
            number of entries in parameter (could be array) not yet implemented
            so I left a placeholder...don't use this byte
        byte n+3:
            bytes forming the parameter's value
    """
    _type_dict = {BooleanType: '\x00', IntType: '\x01', FloatType: '\x02',
        StringType: '\x03', None: '\xff'}
    _inv_type_dict = {'\x00': BooleanType, '\x01': IntType, '\x02': FloatType,
        '\x03': StringType, '\xff': None}

    def __init__(self, pkt=None):
        """ Constructor for the ParameterPacket super-class

        Arguments:
            pkt: optionally a packet to construct us from

        Returns:
            class instance
        """
        super(ParameterPacket, self).__init__()

        if pkt is None:
            self.packet_type(0x00)
            self._param_name = None
            self._param_type = None
        else:
            assert pkt.packet_type() is 0x00, "input packet is of wrong type"
            self._param_name = pkt.payload[0:pkt.payload.index('\xff')]
            type_byte = pkt.payload[pkt.payload.index('\xff') + 1]
            assert type_byte in self._inv_type_dict, 'Param type is not valid'
            self._param_type = self._inv_type_dict[type_byte]

        self._param_value = ''

    @property
    def param_name(self):
        """ Get the parameter's name
        """
        # we identify the division between parameter name and value by \xff
        # since the name goes first and never includes \xff (not a valid ascii
        # character, required for ros params) we know the first time it
        # occurs will be the break
        return self._param_name

    @param_name.setter
    def param_name(self, name):
        """ Set the parameter value

        Arguments:
            param_name: the name of the parameter, if None, then it will call
            the property and return the current name

        Returns:
            No returns if we set the param name
        """
        self._param_name = name
        self._rebuild_packet()

    @property
    def param_type(self):
        """ Get the parameter's type
        """
        return self._param_type

    def _rebuild_packet(self):
        """ Rebuild the packet from its parts (run this after anything is changed)
        """
        self.payload = (self._param_name + '\xff' +
            self._type_dict[self._param_type] + '\x01' + self._param_value)


class StringParameterPacket(ParameterPacket):

    def __init__(self, pkt=None):
        """ Constructor for String parameter packet

        Arguments:
            pkt: optional, if specified then the parameter name, value, and
                type information will be set from  the payload of this packet
                The packet MUST be of type 0x00 and contain the appropriate
                parameter type byte

        Returns:
            object instance
        """
        super(StringParameterPacket, self).__init__(pkt)

        if pkt is not None:
            assert pkt.payload[pkt.payload.index('\xff')+1] == \
                    self._type_dict[StringType], 'packet not a string param'
            self._param_value = pkt.payload[pkt.payload.index('\xff')+3:]
            self._rebuild_packet()
        else:
            self._param_type = StringType

    @property
    def param_value(self):
        """ get the parameter value

        Arguments:
            no arguments

        Returns:
            value: string containng the parameter's value
        """
        return self._param_value

    @param_value.setter
    def param_value(self, value):
        """ Set the parameter's value

        Arguments:
            value: string specifying the parameter value

        Returns:
            if param_value is not specified will return the current internal
            value, elsewise no returns
        """
        assert type(value) is StringType, 'input value must be a string'
        self._param_value = value
        self._rebuild_packet()

class IntParameterPacket(ParameterPacket):

    def __init__(self, pkt=None):
        """ Constructor for Int parameter packet

        Always treated as signed integer 64

        Arguments:
            pkt: optional, if specified then the parameter name and value will
                be picket up from the payload of this packet. It MUST however
                be of type 0x00 and contain the appropriate prameter type byte

        Returns:
            opject instance
        """
        super(IntParameterPacket, self).__init__(pkt)

        if pkt is not None:
            assert pkt.payload[pkt.payload.index('\xff')+1] == \
                self._type_dict[IntType], 'packet not an int param'
            self._param_value = pkt.payload[pkt.payload.index('\xff')+3:]
            self._rebuild_packet()
        else:
            self._param_type = IntType

    @property
    def param_value(self):
        """ Get the parameter value

        Arguments:
            no arguments

        Returns:
            value: int containing parameter's value
        """
        return unpack('q', self._param_value)[0]

    @param_value.setter
    def param_value(self, value):
        """ Set the parameter value

        Arguments:
            value: integer specifying parameter value

        Returns:
            no returns
        """
        assert type(value) is IntType, 'input value must be an integer'
        self._param_value = pack('q', value)
        self._rebuild_packet()

class FloatParameterPacket(ParameterPacket):

    def __init__(self, pkt=None):
        """ Constructor for float parameter packet

        always treated as double

        Arguments:
            pkt: optional, if specified then parameter name and value will be
                picked up from the payload of this packet, but it must of type
                0x00 and contain the appropriate parameter type

        Returns:
            object instance
        """
        super(FloatParameterPacket, self).__init__(pkt)

        if pkt is not None:
            assert pkt.payload[pkt.payload.index('\xff')+1] == \
                self._type_dict[FloatType], 'packet not a float param'
            self._param_value = pkt.payload[pkt.payload.index('\xff')+3:]
            self._rebuild_packet()
        else:
            self._param_type = FloatType

    @property
    def param_value(self):
        """ get the parameter value

        Arguments:
            no arguments

        Returns:
            value: float contianing parameter's value
        """
        return unpack('d', self._param_value)[0]

    @param_value.setter
    def param_value(self, value):
        """ get the parameter value

        Arguments:
            value: integer specifying parameter value

        Returns:
            no returns
        """
        assert type(value) is FloatType, 'input value must be a float'
        self._param_value = pack('d', value)
        self._rebuild_packet()

class BoolParameterPacket(ParameterPacket):

    def __init__(self, pkt=None):
        """ Constructor for boolean parametr packet

        Arguments:
            pkt: optional, if specified then the parameter name and value will
                be picked up from the payload of this packet, but it must be
                of type 0x00 and contain teh appropriate parameter type

        Returns:
            object instance
        """
        super(BoolParameterPacket, self).__init__(pkt)

        if pkt is not None:
            assert pkt.payload[pkt.payload.index('\xff')+1] == \
                self._type_dict[BooleanType], 'packet not a bool param'
            self._param_value = pkt.payload[pkt.payload.index('\xff')+3:]
            self.rebuild_packet()
        else:
            self._param_type = BooleanType

    @property
    def param_value(self):
        """ get the paramter value

        Arguments:
            no arguments

        Returns:
            value: bool containing paramter's value
        """
        return unpack('?', self._param_value)[0]

    @param_value.setter
    def param_value(self, value):
        """ set the paramter value

        Arguments:
            value: bool specifying the parameter value

        Returns:
            no returns
        """
        assert type(value) is BooleanType, 'input value must be a boolean'
        self._param_value = pack('?', value)
        self._rebuild_packet()
