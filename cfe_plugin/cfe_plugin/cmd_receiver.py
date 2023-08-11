"""
.. module:: cfe_ros2_bridge_plugin.cfe_plugin.cmd_receiver
   :synopsis: Class that listens on a port for command data

.. moduleauthor:: Tod Milam

"""
import threading
import socket
import rclpy
import importlib

from struct import unpack
import codecs
import juicer_util.juicer_interface


class CmdReceiver():
    """This class listens on a port for incoming commands.

    Attributes
    ----------
    node : rosnode
        the ROS2 node
    cmd_code_map : dict
        mapping of the command key to the command code
    juicer_interface : JuicerInterface
        the interface to the juicer database
    port : int
        the port number to listen on for command data
    msg_pkg : str
        the ROS2 package containing the ROS2 message structures
    cmd_map : dict
        mapping of the command message id to the command key
    logger : node logger
        used as a shortcut to accessing the logger
    recv_buff_size : int
        the size of the buffer in the receiving socket
    running : bool
        set to True until time for this receiver to quit
    recv_thread : Thread
        the thread running this receiver
    current_value : dict
        the latest value received for each command message

    Methods
    -------
    stop_thread():
        Tells the thread to stop and cleanup.
    receive_thread():
        Starts the thread processing, listening for commands.
    handle_packet(datagram):
        Process an incoming data packet.
    get_buffered_data(key, clear):
        Returns the buffered command value for the given key.
    get_pkt_id(datagram):
        Return the packet id for the cFE data packet.
    get_seq_count(datagram):
        Return the sequence count for the cFE data packet.
    """
    def __init__(self, node, msg_pkg, port, command_info, juicer_interface):
        '''
        Initializes the attributes of the cmd receiver object.

        Args:
            node (rosnode): The ROS2 node
            msg_pkg (str): The ROS2 package containing the message structures
            port (int): The port number to listen on
            command_info (): The command configuration information
            juicer_interface (JuicerInterface): The interface to the Juicer database
        '''
        self._node = node
        self._cmd_code_map = {}
        self._juicer_interface = juicer_interface
        self._port = port
        self._msg_pkg = msg_pkg
        self._cmd_map = {}
        self._logger = self._node.get_logger()
        for cmd in command_info:
            self._node.get_logger().debug("type: " + str(cmd))
            self._node.get_logger().debug("  cfe_mid: " + str(command_info[cmd]['cfe_mid']))
            self._node.get_logger().debug("  cmd_code: " + str(command_info[cmd]['cmd_code']))
            self._cmd_map[command_info[cmd]['cfe_mid']] = cmd
            self._cmd_code_map[cmd] = command_info[cmd]['cmd_code']
        self._logger.debug("command map is " + str(self._cmd_map))
        self._recv_buff_size = 4096

        self._running = True
        self._recv_thread = threading.Thread(target=self.receive_thread)

        self._logger.info("starting thread to receive CFS command")
        self._recv_thread.start()
        self._latest_values = {}

    def stop_thread(self):
        '''
        Tells the thread to stop and cleanup.
        '''
        self._running = False
        self._recv_thread.join()

    def receive_thread(self):
        '''
        Starts the thread processing, listening for commands.
        '''
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._logger.info("binding to port " + str(self._port))
        self._sock.bind(("", self._port))

        self._socket_err_count = 0
        while self._running:
            try:
                # receive message
                datagram, host = self._sock.recvfrom(self._recv_buff_size)

                # ignore data if not long enough (doesn't contain header)
                if len(datagram) < 6:
                    continue

                self.handle_packet(datagram)

            except socket.error:
                self._logger.warn("Error receiving command data.")

    def handle_packet(self, datagram):
        '''
        Process an incoming data packet.

        Args:
            datagram (bytearray): The incoming cFE data packet
        '''
        packet_id = self.get_pkt_id(datagram)
        seq = self.get_seq_count(datagram)
        self._logger.debug("Sequence count = " + str(seq))
        if packet_id in self._cmd_map:
            self._logger.debug("Handling command message id " + packet_id)
            ros_name = self._cmd_map[packet_id]
            self._logger.debug("Received packet for " + ros_name)
            MsgType = getattr(importlib.import_module(self._msg_pkg + ".msg"),
                              self._cmd_map[packet_id])
            msg = MsgType()
            self._juicer_interface.parse_packet(datagram, 0, self._cmd_map[packet_id], msg, self._msg_pkg)

            # check to see if we have telem data for this key. if not create a new list
            if ros_name not in self._latest_values:
                self._latest_values[ros_name] = []

            # append latest message to the list for this key
            self._latest_values[ros_name].append(msg)

            self._logger.debug("Parsed cmd: " + str(msg))
        else:
            self._logger.info("Don't know how to handle command message id " + packet_id)

    def get_buffered_data(self, key, clear):
        '''
        Returns the buffered command value for the given key.

        Args:
            key (str): The key for the command
            clear (bool): Flag indicating if data should be cleared once returned

        Returns:
            current_value (): The command value for the key
        '''
        retval = None
        if key in self._latest_values:
            retval = self._latest_values[key]
        if clear and (key in self._latest_values):
            del self._latest_values[key]
        return retval

    @staticmethod
    def get_pkt_id(datagram):
        '''
        Return the packet id for the cFE data packet.

        Args:
            datagram (bytearray): The cFE data packet

        Returns:
            pkt_id (int): The packet id of the data packet
        '''
        streamid = unpack(">H", datagram[:2])
        return hex(streamid[0])

    @staticmethod
    def get_seq_count(datagram):
        '''
        Return the sequence count for the cFE data packet.

        Args:
            datagram (bytearray): The cFE data packet

        Returns:
            seq_count (int): The sequence count
        '''
        streamid = unpack(">H", datagram[2:4])
        return streamid[0] & 0x3FFF  # # sequence count mask
