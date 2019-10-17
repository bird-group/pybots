import pdb

import socket

class SocketManager(object):
    """ A class to handle socket connections
    """

    def __init__(self, ip='', port='', buffer_size=''):
        """ Constructor for the TCP socket connection.

        Arguments:
            ip: ip address of the client we want, '' for anyone
            port: port on which we're going to talk
            buffer_size: size of the input buffer

        Returns:
            the object
        """
        super(SocketManager, self).__init__()
        self._ip_addr = ip
        self._port = port
        self._buffer_size = buffer_size
        self.status = False
        self._conn = None
        self.remote_addr = None

    def start_send(self):
        """ Connect to the socket as a client.

        Arguments:
            no arguments

        Returns:
            no returns
        """
        self.sock.connect((self._ip_addr, self._port))
        self.status = True

    def start_recv(self):
        """ Accept an incoming connection

        Arguments:
            no arguments

        Returns:
            no returns
        """
        self._conn, self.remote_addr = self.sock.accept()
        self.status = True

    def stop(self):
        """ Close the socket connection

        Arguments:
            no arguments

        Returns:
            no returns
        """
        self.sock.close()
        self.status = False

class TCPSocket(SocketManager):
    """ a class to handle tcp socket connections
    """

    def __init__(self, ip = '', port = '', buffer_size = ''):
        """ Constructor for the TCP socket connection.

        Arguments:
            ip: ip address of the client we want, '' for anyone
            port: port on which we're going to talk
            buffer_size: size of the input buffer

        Returns:
            the object
        """
        super(TCPSocket, self).__init__(ip, port, buffer_size)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    def start_send(self):
        """ Connect to the socket as a client.

        Arguments:
            no arguments

        Returns:
            no returns
        """
        self.sock.connect((self._ip_addr, self._port))
        self.status = True

    def start_recv(self):
        """ Accept an incoming connection

        Arguments:
            no arguments

        Returns:
            no returns
        """
        self._conn, self.remote_addr = self.sock.accept()
        self.status = True

    def start_listen(self):
        """ Bind the socket as the server

        Arguments:
            no arguments

        Returns:
            no returns
        """
        self.sock.bind((self._ip_addr, self._port))
        self.sock.listen(1)

    def write(self, data):
        """ Send data on the socket, take action if it has closed.

        Arguments:
            data: char array of data to send

        Returns:
            is_succesful: boolean indicating success
        """
        try:
            self._conn.send(data)
            return True
        except:
            return False

    def receive_safe(self):
        """ receive from socket, take action if it is closed

        Arguments:
            no arguments

        Returns:
            data: byte array of the received data, will return false on failure
        """
        try:
            data = self.conn.recv(self._buffer_size)
            return data
        except:
            return False

class UDPSocket(SocketManager):
    """ a class to handle udp socket connections
    """

    def __init__(self, ip='', port='', buffer_size='', dest_ip='', dest_port=''):
        """ Constructor for the TCP socket connection.

        Arguments:
            ip: ip address of the client we want, '' for anyone
            port: port on which we're going to talk
            buffer_size: size of the input buffer
            dest_ip: the ip to send to
            dest_port: the port to send to

        Returns:
            the object
        """
        super(UDPSocket, self).__init__(ip, port, buffer_size)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.dest_ip = dest_ip
        self.dest_port = dest_port

    def start_listen(self):
        """ Bind the socket as the server

        Arguments:
            no arguments

        Returns:
            no returns
        """
        self.sock.bind((self._ip_addr, self._port))

    def write(self, data):
        """ Send data on the socket, take action if it has closed.

        Arguments:
            data: char array of data to send

        Returns:
            is_succesful: boolean indicating success
        """
        try:
            self.sock.sendto(data, (self.dest_ip, self.dest_port))
            return True
        except:
            return False

    def receive_safe(self):
        """ receive from socket, take action if it is closed

        Arguments:
            no arguments

        Returns:
            data: byte array of the received data, will return false on failure
        """
        try:
            data = self.sock.recv(self._buffer_size)
            return data
        except:
            return False
