""" AVIA protocol implementations

Base Class AVIaParser
    contains shared functions
Inherited Classes
    AVIaSerial
        Serial-specific implementation details
    AVIaTCP
        TCP-specific implementation details
"""
import os

import serial
import copy

from struct import unpack
from common_packets.avia_packet import AVIaPacket
from communications.socket_connections import UDPSocket


class AVIaParser(object):
    """ class to handle low-level work of processing AVIa packets out
    of a data stream from some input source

    This is a worker super-class, you should use the inherited classes
    which implement a specific interface (TCP or serial)
    """
    def __init__(self):
        """ constructor

        intializes the parser variable

        Args:
            no args

        Returns:
            AVIaParser instance
        """
        self._recv_packet = AVIaPacket()
        self.packet_buffer = []
        self.buf = []
        self.is_new_packet = False
        self.num_packets = 0
        self.comm_object = None

    def parse_buffer(self, n_lim=6):
        """ parse the buffer to find packets

        searches buffer for packets, returns the number of packets found

        Arguments:
            n_lim: the maximum number of packets that will be read in one go

        Returns:
            num_packets: the number of packets found

        The packets will be found in the packet_buffer field of the
        object
        """

        self.num_packets = 0
        self.is_new_packet = False

        for i in range(0, n_lim):
            if len(self.buf) < 8:
                return self.num_packets

            # search for the header...
            looking_for_header = True
            while looking_for_header and (len(self.buf) >= 8):
                header = 0
                try:
                    first = self.buf.index("A")
                    self.buf = self.buf[first:]
                    header += (self.buf.index("V"))
                    header += (self.buf.index("I"))
                    header += (self.buf.index("a"))
                except ValueError:
                    return self.num_packets
                if header != 6:
                    self.buf = self.buf[(first + 1):]
                else:
                    looking_for_header = False

            # check to see if there's enough bytes left to make a packet
            if len(self.buf) < (8):
                return self.num_packets

            # get the start of the payload, then the packet length
            start_index = self.buf.index("a") + 3
            pkt_len = ord(self.buf[start_index - 2])

            # check to make there are enough bytes to finish the packet
            if len(self.buf) < (8 + pkt_len):
                return self.num_packets

            # build an avia packet with the data
            self._recv_packet.packet_type(
                self.buf[start_index-1])
            self._recv_packet.payload = ''.join(
                self.buf[start_index:start_index+pkt_len])
            cksum = ''.join(
                self.buf[start_index + pkt_len:start_index + pkt_len+2])
            if len(cksum) < 2:
                # return if we ran out of bytes
                return self.num_packets

            # compute and verify the checksum, if it verifies, add the
            # packet to our buffer
            cksum = unpack('h', cksum)
            if cksum[0] == self._recv_packet.check_sum():
                self.packet_buffer.append(
                    copy.deepcopy(self._recv_packet))
                self.is_new_packet = True
                self.num_packets += 1
            self.buf = self.buf[(start_index + pkt_len + 2):]

        # return the number of packets found
        return self.num_packets

    def write_bytes(self, output_buffer=None):
        """ Write a buffer or packet out on this interface

        Arguments:
            output_buffer:  avia packet or a list of bytes

        Returns:
            no returns:
        """
        if output_buffer is None or self.comm_object is None:
            return
        if isinstance(output_buffer, AVIaPacket):
            self.comm_object.write(output_buffer.full_packet())
        if type(output_buffer) is list:
            self.comm_object.write(''.join(output_buffer[0:]))

    def clear_packet_buffer(self):
        """Clear the packet buffer

        Arguments:
            no arguments

        Returns:
            no returns
        """
        self.packet_buffer = []

class AVIaSerial(AVIaParser):
    """ AVIa interface for serial comms
    """
    def __init__(self, comm_port, baud_rate, dump_file=None):
        """ constructor

        Create the AVIaSerial parser as an instance of a class
        inheriting from the AVIaParser

        Args:
            comm_port: path to the comm port to use
            baud_rate: the baud to use on that comm port
            dump_file: optional, if specified then all data read in will be
                written to this file

        Returns:
            instance of the class
        """
        super(AVIaSerial, self).__init__()
        self.comm_object = serial.Serial(comm_port, baud_rate)

        # we're going to save up data and dump it 100 bytes at a time
        self._dump_data = []
        self._dump_path = dump_file

    def __enter__(self):
        """ on create, don't do anything
        """
        return self

    def __exit__(self, type, value, traceback):
        """ destructor, make sure the serial port gets closed
        """
        self.comm_object.close()

    def service_comms(self):
        """ read bytes and look for packets

        reads in all available packets from the serial interface and
        examines the input buffer for avia packets

        Args:
            no args

        Returns:
            num_packets: the number of packets found in the packet
        """
        new_data = self.comm_object.read(self.comm_object.inWaiting())

        self._dump_data.extend(new_data)
        if len(self._dump_data) > 100 and self._dump_path:
            with open(self._dump_path, 'a+') as dump_file:
                dump_file.write(''.join(self._dump_data))
            self._dump_data = []

        self.buf.extend(new_data)
        if len(self.buf) >= 8:
            return self.parse_buffer()
        else:
            return 0

    def read_bytes(self):
        """ Alias to service_comms. Exists for compatibility as we migrate to
        everyone using the service_comms function
        """
        self.service_comms()


class AVIaTCP(AVIaParser):
    """ AVIa interface for tcp comms.
    """
    def __init__(self, comm_sock=None):
        """ constructor

        Create the AVIaTCP parser as an instance of a class
        inheriting from the AVIaParser

        Args:
            comm_sock: socket interface to use for reading bytes

        Returns:
            instance of the class
        """
        super(AVIaTCP, self).__init__()
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


class AVIaUDP(AVIaParser):
    """ AVIa interface for udp comms.
    """
    def __init__(self, comm_sock=None):
        """ constructor

        Create the AVIaUDP parser as an instance of a class
        inheriting from the AVIaParser

        Args:
            comm_sock: socket interface to use for reading bytes

        Returns:
            instance of the class
        """
        super(AVIaUDP, self).__init__()
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
        buffer for avia packets

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

    def read_bytes(self):
        """ Alias to service_comms. Exists for compatibility as we migrate to
        everyone using the service_comms function
        """
        self.service_comms()


class AVIaFile(AVIaParser):
    """ AVIa interface for dump files.
    """
    def __init__(self):
        """ constructor

        Create the AVIaFile parser as an instance of a class
        inheriting from the AVIaParser

        Arguments:
            no arguments

        Returns:
            instance of the class
        """
        super(AVIaFile, self).__init__()

    def __enter__(self):
        """ on create, don't do anything
        """
        return self

    def __exit__(self, type, value, traceback):
        """ destructor
        """
        return

    def parse_file(self, file_path):
        """ Parse all avia packets out of a file

        Arguments:
            file_path: the file to be parsed

        Returns:
            packets: tuple of all the packets
        """
        assert os.path.isfile(file_path), 'must specify valid file'
        with open(file_path, 'rb') as dump_file:
            self.buf = dump_file.read()
        self.parse_buffer(n_lim=10000)
        return tuple(self.packet_buffer)
