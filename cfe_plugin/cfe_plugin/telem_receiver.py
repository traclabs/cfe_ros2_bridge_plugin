import threading
import socket
import rclpy
import importlib

from struct import unpack
import codecs
import juicer_util.juicer_interface


class TelemReceiver():
    def __init__(self, node, msg_pkg, port, telem_info, juicer_interface):
        self._node = node
        self._ros_topic_map = {}
        self._juicer_interface = juicer_interface
        self._port = port
        self._msg_pkg = msg_pkg
        self._tlm_map = {}
        self._logger = self._node.get_logger()
        for tlm in telem_info:
            self._node.get_logger().debug("type: " + str(tlm))
            self._node.get_logger().debug("  cfe_mid: " + str(telem_info[tlm]['cfe_mid']))
            self._node.get_logger().debug("  topic_name: " + telem_info[tlm]['topic_name'])
            self._tlm_map[telem_info[tlm]['cfe_mid']] = tlm
            self._ros_topic_map[tlm] = telem_info[tlm]['topic_name']
        self._logger.debug("telem map is " + str(self._tlm_map))
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

                self.handle_packet(datagram)

            except socket.error:
                self._logger.warn("Error receiving telemetry data.")

    def handle_packet(self, datagram):
        packet_id = self.get_pkt_id(datagram)
        if packet_id in self._tlm_map:
            ros_name = self._tlm_map[packet_id]
            self._logger.info("Received packet for " + ros_name)
            MsgType = getattr(importlib.import_module(self._msg_pkg + ".msg"),
                              self._tlm_map[packet_id])
            msg = MsgType()
            setattr(msg, "seq", self.get_seq_count(datagram))
            self._juicer_interface.parse_packet(datagram, 0, self._tlm_map[packet_id], msg, self._msg_pkg)
            self._current_value[ros_name] = msg
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
