import socket
import rclpy
import importlib

from struct import unpack
import codecs
import juicer_util.juicer_interface
from std_msgs.msg import Header


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
        self._timer_period = 0.05
        self._logger.info("starting thread to receive CFS telemetry")
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.bind(("", self._port))
        self._recv_timer = self._node.create_timer(self._timer_period, self.receive_callback)
        self._latest_values = {}

    def stop_thread(self):
        self._running = False
        self._recv_timer.shutdown();

    def receive_callback(self):
        if self._running:
            # self._logger.info("telem_reciever.callback() time: " + str(self._node.get_clock().now()))
            try:
                # receive message
                datagram, host = self._sock.recvfrom(self._recv_buff_size)

                # ignore data if not long enough (doesn't contain header)
                if len(datagram) < 6:
                    return

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
        retval = None
        if key in self._latest_values:
            retval = self._latest_values[key]
        if clear and (key in self._latest_values):
            del self._latest_values[key]
        return retval

    @staticmethod
    def get_pkt_id(datagram):
        streamid = unpack(">H", datagram[:2])
        return hex(streamid[0])

    @staticmethod
    def get_seq_count(datagram):
        streamid = unpack(">H", datagram[2:4])
        return streamid[0] & 0x3FFF  # # sequence count mask
