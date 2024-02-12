"""
.. module:: cfe_ros2_bridge_plugin.cfe_plugin.telem_receiver
   :synopsis: Class that listens on a port for telemetry data

.. moduleauthor:: Tod Milam

"""
import socket
import rclpy
import importlib
import sys

from struct import unpack
import codecs
import juicer_util.juicer_interface
from std_msgs.msg import Header
from cfe_msgs.msg import BinaryPktPayload  # Default binary packet format

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
        node.get_logger().debug(f"TelemReceiver(telem_info[{str(len(telem_info))})")
        self._node = node
        self._ros_topic_map = {}
        self._juicer_interface = juicer_interface
        self._sock = sock
        self._msg_pkg = msg_pkg
        self._tlm_map = {}
        self._key_map = {}
        self._logger = self._node.get_logger()

        for tlm in telem_info:
            self._tlm_map[telem_info[tlm]['cfe_mid']] = telem_info[tlm]['structure']
            self._key_map[telem_info[tlm]['cfe_mid']] = str(tlm)
            self._ros_topic_map[tlm] = telem_info[tlm]['topic_name']

        self._recv_buff_size = 4096
        self._timer_period = 0.05  # as long as data from cFS is coming in, per MID,
                                   # slower than 20hz, this should be ok

        self._logger.info("starting timer thread to receive CFS telemetry")
        self._recv_timer = self._node.create_timer(self._timer_period, self.receive_callback)

        self._latest_values = {}

    def receive_callback(self):

        # if it takes too long to process the socket buffer, it will slow down the
        # thread updates.  This will give us a warning if that happens
        time_since_last_call = self._recv_timer.time_since_last_call() * 1e-9
        if time_since_last_call >= self._timer_period:
            self._logger.warn("TelemReceiver update thread not able to process data within "
                + str(1.0 / self._timer_period)
                + " Hz update rate, slow downs may occurr")

        # while loop makes sure we process all the socket data before moving on
        while True:
            try:
                # receive message
                datagram, addr = self._sock.recvfrom(self._recv_buff_size)
                if len(datagram) < 6:
                    return

                self._logger.debug("Incoming data is length " + str(len(datagram)))
                self.handle_packet(datagram)

            except socket.error:
                return

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

            if not ros_name:
                # Packet is defined in configuration, but no structure was specified, implying we should use default binary encoding
                self._logger.warn("Received packet for pid without struct name! Handling Under testing: " + str(packet_id))

                #@ Create Generic Binary Packet containing payload
                # ROS doesn't seem to allow a msg definition of a simple bytes array, so we must split it
                #  into an array of individual byte objects (byte[])
                # VERIFY: Is this correct and/or the most efficient way to do this?
                tmp_data = datagram[16:]
                msg_data = [i.to_bytes(1, sys.byteorder) for i in tmp_data]
                msg = BinaryPktPayload(data=msg_data)
            else:
                
                MsgType = getattr(importlib.import_module(self._msg_pkg + ".msg"),
                                  ros_name)
                msg = MsgType()
                self._juicer_interface.parse_packet(datagram, 0, self._tlm_map[packet_id], msg, self._msg_pkg)

            # Populate common header attributes as appropriate
            msg_attrs = dir(msg)
            if "seq" in msg_attrs:
                setattr(msg, "seq", self.get_seq_count(datagram))
            if "header" in msg_attrs:
                hdr = Header()
                hdr.stamp = mytime
                setattr(msg, "header", hdr)
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
        Returns the latest telemetry value for the given key.

            Parameters:
                    key (str): The key for the telemetry
                    clear (bool): If queue should be cleared

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
        return streamid[0] & 0x3FFF  # sequence count mask
