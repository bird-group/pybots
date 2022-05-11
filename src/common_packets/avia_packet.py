""" the avia packet definition
"""
from struct import pack
from struct import unpack

class AVIaPacket(object):

    """ class to handle the avia packet low-level work
    """

    def __init__(self):
        """ constructor

        sets up the avia packet to be empty with type 0

        Args:
            no args

        Returns:
            an instance of avia packet
        """
        self.pkt_type = 0
        self.payload = []

    def __len__(self):
        """ len method for AVIaPacket

        defines the builtin len method to return the size of the payload

        Args:
            no args

        Returns:
            packet_length: the number of bytes in the packet
        """
        return len(self.payload)

    def check_sum(self):
        """ compute the checksum

        the checksum is the bitwise AND of the packet type and each byte
        in the payload, represented as a uint16

        Args:
            no args

        Returns:
            the checksum
        """
        if type(self.pkt_type) is str:
            cksum = ord(self.pkt_type) + len(self.payload)
        else:
            cksum = self.pkt_type + len(self.payload)
        for byte in self.payload:
            if type(byte) is str:
                cksum += ord(byte)
            else:
                cksum += byte
        return cksum

    def packet_type(self, new_type=None):
        """ set or retrieve packet type

        used to set the type of a packet or retrieve it, depending on
        input args

        Args:
            new_type: optionally, the type to set the packet type to

        Returns:
            pkt_type: if new_type is None, retrieves the packet type from
                the packet data
        """
        if new_type is None:
            if type(self.pkt_type) is str:
                return unpack('B', self.pkt_type)[0]
            elif type(self.pkt_type) is int:
                return self.pkt_type
        elif type(new_type) is str:
            self.pkt_type = new_type
        elif type(new_type) is int:
            self.pkt_type = chr(new_type)

    def full_packet(self):
        """ retrieves the full packet

        makes a char array of the packet for writing on serial.

        Args:
            no args

        Returns:
            out: char array of header, length, type, payload, checksum
        """
        out = ['A', 'V', 'I', 'a']
        out.append(chr(len(self.payload)))
        out.append(chr(self.packet_type()))
        if type(self.payload[0]) is str:
            out.extend(self.payload)
        else:
            for byte in self.payload:
                out.extend(chr(byte))
        out.extend(pack('h', self.check_sum())[0:])
        out = ''.join(out[0:])
        return out

    @property
    def type(self):
        """Getter for packet type

        Arguments:
            no arguments

        Returns:
            type: packet type identifier
        """
        return unpack('B', self.pkt_type)[0]
