""" flightgear protocol implementations

Base Class FlightGearParser
    contains shared functions
Inherited Classes
    FlightGearTCP
        TCP-specific implementation details
    FlightGearUDP
        UDP-specific implementation details
"""
import pdb
import copy

from struct import unpack
from common_packets.avia_packet import AVIaPacket
from communications.socket_connections import UDPSocket

class FlightGearParser(object):
    """ class to handle low-level work of processing FlightGear packets out
    of a data stream from some input source

    This is a worker super-class, you should use the inherited classes
    which implement a specific interface (UDP or TCP)
    """
    def __init__(self):
        """ constructor

        intializes the parser variable

        Args:
            no args

        Returns:
            FlightGearParser instance
        """
        self._recv_packet = []
        self.packet_buffer = []
        self.buf = []
        self.is_new_packet = False
        self.num_packets = 0
        self.comm_object = None

    def parse_buffer(self):
        """ parse the buffer to find packets

        searches buffer for packets, returns the number of packets found

        Args:
            no args

        Returns:
            num_packets: the number of packets found

        The packets will be found in the packet_buffer field of the
        object
        """

        # Flightgear packets can have all kinds of structure, bure here we're
        # assuming that they consist of a tab-delimited set of fields followed
        # by a newline character

        self.num_packets = 0
        self.is_new_packet = False

        for i in range(0, 6):
            if len(self.buf) < 8:
                return self.num_packets

            # newline marks the end of the packet
            try:
                packet_end = self.buf.index("\n")
            except ValueError:
                return self.num_packets

            # separate all of the fields preceeding the packet end
            packet_data = self.buf[0:packet_end]
            packet = []
            getting_data = True
            while getting_data:
                try:
                    field_end = packet_data.index("\t")
                    packet.append(''.join(packet_data[0:field_end]))
                    packet_data = packet_data[(field_end + 1):]
                except ValueError:
                    packet.append(''.join(packet_data))
                    getting_data = False
                    continue

            self.packet_buffer.append(
                copy.deepcopy(packet))
            self.is_new_packet = True
            self.num_packets += 1
            self.buf = self.buf[(packet_end + 1):]

        # return the number of packets found
        return self.num_packets

    def write_bytes(self, output_buffer=None):
        """ Write a buffer or packet out on this interface

        Arguments:
            output_buffer: list of values to write. Each element in the list
                must either be a string or convertible to one using str(el)

        Returns:
            no returns:
        """
        if output_buffer is None or self.comm_object is None:
            return
        output_bytes = ''
        for element in output_buffer:
            if isinstance(element, str):
                output_bytes += element
            else:
                output_bytes += str(element)
            output_bytes += '\t'
        output_bytes += '\n'
        self.comm_object.write(output_bytes)

class FlightGearTCP(FlightGearParser):
    """ AVIa interface for tcp comms.
    """
    def __init__(self, comm_sock=None):
        """ constructor

        Create the FlightGear parser as an instance of a class
        inheriting from the FlightGearParser

        Args:
            comm_sock: socket interface to use for reading bytes

        Returns:
            instance of the class
        """
        super(FlightGearTCP, self).__init__()
        self.comm_object = comm_sock

    def __enter__(self):
        """ on create, don't do anything
        """
        return self

    def __exit__(self, type, value, traceback):
        """ destructor, make sure the socket gets closed
        """
        self.comm_object.stop()

    def service_comms(self):
        """ Read bytes and look for packets

        reads in all available data from the tcp socket and examines the input
        buffer for avia packets

        Args:
            no args

        Returns:
            num_packets: the number of packets found in the packet
        """
        if self.comm_object is not None:
            len_buf = len(self.buf)
            self.buf.extend(self.comm_object.outgoing_sock.recv(1024))
            if len_buf == len(self.buf):
                return None
            if len(self.buf) >= 8:
                return self.parse_buffer()
            else:
                return 0

    def read_bytes(self):
        """ Alias to service_comms. Exists for compatibility as we migrate to
        everyone using the service_comms function
        """
        self.service_comms()

class FlightGearUDP(FlightGearParser):
    """ FlightGear interface for udp comms.
    """
    def __init__(self, comm_sock=None):
        """ constructor

        Create the FlightGearUDP parser as an instance of a class
        inheriting from the FlightGearParser

        Args:
            comm_sock: socket interface to use for reading bytes

        Returns:
            instance of the class
        """
        super(FlightGearUDP, self).__init__()
        assert isinstance(comm_sock, UDPSocket) or comm_sock is None,\
            "comm socket must be UDP!"
        self.comm_object = comm_sock

    def __enter__(self):
        """ on create, don't do anything
        """
        return self

    def __exit__(self, type, value, traceback):
        """ destructor, make sure the socket gets closed
        """
        self.comm_object.stop()

    def service_comms(self):
        """ Read bytes and look for packets

        reads in all available data from the tcp socket and examines the input
        buffer for flight gear packets

        Args:
            no args

        Returns:
            num_packets: the number of packets found in the packet
        """
        if self.comm_object is not None:
            len_buf = len(self.buf)
            self.buf.extend(self.comm_object.receive_safe())
            if len_buf == len(self.buf):
                return None
            if len(self.buf) >= 9:
                return self.parse_buffer()
            else:
                return 0
