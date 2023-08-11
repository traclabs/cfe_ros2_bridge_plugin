"""
.. module:: cfe_ros2_bridge_plugin.cfe_plugin.telem_receiver
   :synopsis: Class that listens on a port for telemetry data

.. moduleauthor:: Tod Milam

"""
import threading
import socket
import rclpy
import importlib

from struct import unpack
import codecs
import juicer_util.juicer_interface
from std_msgs.msg import Header


class TelemReceiver():
    """This class listens on a port for incoming telemetry.

    Attributes
    ----------
    node : rosnode
        the ROS2 node
    ros_topic_map : dict
        mapping of telem message ID to ROS topic
    juicer_interface : JuicerInterface
        the interface to the juicer database
    port : int
        the port number to listen on for telemetry data
    msg_pkg : str
        the ROS2 package containing the ROS2 message structures
    tlm_map : dict
        mapping of telem message ID to ROS data structure
    key_map : dict
        mapping of telem message ID to telemetry key (from config file)
    logger : node logger
        used as a shortcut to accessing the logger
    recv_buff_size : int
        the size of the buffer in the receiving socket
    running : bool
        set to True until time for this receiver to quit
    recv_thread : Thread
        the thread running this receiver
    current_value : dict
        the latest value received for each piece of telemetry

    Methods
    -------
    stop_thread():
        Tells the thread to stop and cleanup.
    receive_thread():
        Starts the thread processing, listening for telemetry.
    handle_packet(datagram):
        Process an incoming data packet.
    get_buffered_data(key, clear):
        Returns the buffered telemetry value for the given key.
    get_pkt_id(datagram):
        Return the packet id for the cFE data packet.
    get_seq_count(datagram):
        Return the sequence count for the cFE data packet.
    """
    def __init__(self, node, msg_pkg, port, telem_info, juicer_interface):
        '''
        Initializes the attributes of the telem receiver object.

        Args:
            node (rosnode): The ROS2 node
            msg_pkg (str): The ROS2 package containing the message structures
            port (int): The port number to listen on
            telem_info (): The telemetry configuration information
            juicer_interface (JuicerInterface): The interface to the Juicer database
        '''
        self._node = node
        self._ros_topic_map = {}
        self._juicer_interface = juicer_interface
        self._port = port
        self._msg_pkg = msg_pkg
        self._tlm_map = {}
        self._key_map = {}
        self._logger = self._node.get_logger()
        self._node.get_logger().debug("telem_receiver got these telemetry structs")
        for tlm in telem_info:
            port = telem_info[tlm]['port']
            if port == self._port:
                self._node.get_logger().debug("type: " + str(tlm))
                self._node.get_logger().debug("  structure: " + str(telem_info[tlm]['structure']))
                self._node.get_logger().debug("  cfe_mid: " + str(telem_info[tlm]['cfe_mid']))
                self._node.get_logger().debug("  topic_name: " + telem_info[tlm]['topic_name'])
                self._node.get_logger().debug("  port: " + str(telem_info[tlm]['port']))
                self._tlm_map[telem_info[tlm]['cfe_mid']] = telem_info[tlm]['structure']
                self._key_map[telem_info[tlm]['cfe_mid']] = str(tlm)
                self._ros_topic_map[tlm] = telem_info[tlm]['topic_name']
            # else:
            #     self._node.get_logger().info("Skipping telem on port " + str(port) + " as we're listening to port " + str(self._port))
        self._logger.debug("telem map is " + str(self._tlm_map))
        self._recv_buff_size = 4096

        self._running = True
        self._recv_thread = threading.Thread(target=self.receive_thread)

        self._logger.info("starting thread to receive CFS telemetry")
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
        Starts the thread processing, listening for telemetry.
        '''
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.bind(("", self._port))
        # self._sock = socket.create_server(("", self._port), socket.AF_INET, None, True)

        self._socket_err_count = 0
        while self._running:
            try:
                # receive message
                datagram, host = self._sock.recvfrom(self._recv_buff_size)

                # ignore data if not long enough (doesn't contain header)
                if len(datagram) < 6:
                    continue

                self._logger.debug("Incoming data is length " + str(len(datagram)))
                self.handle_packet(datagram)

            except socket.error:
                self._logger.warn("Error receiving telemetry data.")

    def handle_packet(self, datagram):
        '''
        Process an incoming data packet.

        Args:
            datagram (bytearray): The incoming cFE data packet
        '''
        packet_id = self.get_pkt_id(datagram)
        if packet_id in self._tlm_map:
            # get the current timestamp to add to the message
            mytime = self._node.get_clock().now().to_msg()
            ros_name = self._tlm_map[packet_id]
            self._logger.info("Received packet for " + ros_name + ", pid: " + str(packet_id))
            MsgType = getattr(importlib.import_module(self._msg_pkg + ".msg"),
                              ros_name)
            msg = MsgType()
            msg_attrs = dir(msg)
            if "seq" in msg_attrs:
                setattr(msg, "seq", self.get_seq_count(datagram))
            else:
                self._logger.warn("Failed to find 'seq' in message.")
            if "header" in msg_attrs:
                hdr = Header()
                hdr.stamp = mytime
                setattr(msg, "header", hdr)
                # self._logger.info("Set the header time to " + str(mytime))

            self._juicer_interface.parse_packet(datagram, 0, self._tlm_map[packet_id], msg, self._msg_pkg)
            key = self._key_map[packet_id]

            # check to see if we have telem data for this key. if not create a new list
            if key not in self._latest_values:
                self._latest_values[key] = []

            # append latest message to the list for this key
            self._latest_values[key].append(msg)

        else:
            self._logger.info("Don't know how to handle message id " + packet_id)

    def get_buffered_data(self, key, clear):
        '''
        Returns the buffered telemetry value for the given key.

        Args:
            key (str): The key for the telemetry
            clear (bool): Flag indicating if data should be cleared once returned

        Returns:
            current_value (): The telemetry value for the key
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
