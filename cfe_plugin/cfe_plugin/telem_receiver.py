import threading
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
    def __init__(self, node, msg_pkg, port, telem_info, juicer_interface):
        self._node = node
        self._ros_topic_map = {}
        self._juicer_interface = juicer_interface
        self._port = port
        self._msg_pkg = msg_pkg
        self._tlm_map = {}
        self._key_map = {}
        self._logger = self._node.get_logger()
        self._node.get_logger().debug("telem_receiver got "+str(len(telem_info))+" telemetry structs")
        for tlm in telem_info:
            port = telem_info[tlm]['port']
            if port == self._port:
                #self._node.get_logger().debug("type: " + str(tlm))
                #self._node.get_logger().debug("  structure: " + str(telem_info[tlm]['structure']))
                #self._node.get_logger().debug("  cfe_mid: " + str(telem_info[tlm]['cfe_mid']))
                #self._node.get_logger().debug("  topic_name: " + telem_info[tlm]['topic_name'])
                #self._node.get_logger().debug("  port: " + str(telem_info[tlm]['port']))
                self._tlm_map[telem_info[tlm]['cfe_mid']] = telem_info[tlm]['structure']
                self._key_map[telem_info[tlm]['cfe_mid']] = str(tlm)
                self._ros_topic_map[tlm] = telem_info[tlm]['topic_name']
            # else:
            #     self._node.get_logger().info("Skipping telem on port " + str(port) + " as we're listening to port " + str(self._port))
        #self._logger.debug("telem map is " + str(self._tlm_map))
        self._recv_buff_size = 4096

        self._running = True
        self._recv_thread = threading.Thread(target=self.receive_thread)

        self._logger.info("starting thread to receive CFS telemetry")
        self._recv_thread.start()
        self._current_value = {}

    def stop_thread(self):
        self._running = False
        self._recv_thread.join()

    def receive_thread(self):
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
#            else:
#                self._logger.warn("Failed to find 'seq' in message.")
            if "header" in msg_attrs:
                hdr = Header()
                hdr.stamp = mytime
                setattr(msg, "header", hdr)
                # self._logger.info("Set the header time to " + str(mytime))

            key = self._key_map[packet_id]
            self._current_value[key] = msg
        else:
            self._logger.info("Don't know how to handle message id " + packet_id)

    def get_latest_data(self, key):
        retval = None
        if key in self._current_value:
            retval = self._current_value[key]
        return retval

    @staticmethod
    def get_pkt_id(datagram):
        streamid = unpack(">H", datagram[:2])
        return hex(streamid[0])

    @staticmethod
    def get_seq_count(datagram):
        streamid = unpack(">H", datagram[2:4])
        return streamid[0] & 0x3FFF  # # sequence count mask
