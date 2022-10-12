import threading
import socket
import rclpy
import importlib

from struct import unpack
import codecs
import juicer_util.juicer_interface


class CmdReceiver():
    def __init__(self, node, msg_pkg, port, command_info, juicer_interface):
        self._node = node
        self._cmd_code_map = {}
        self._juicer_interface = juicer_interface
        self._port = port
        self._msg_pkg = msg_pkg
        self._cmd_map = {}
        self._logger = self._node.get_logger()
        for cmd in command_info:
            self._node.get_logger().info("type: " + str(cmd))
            self._node.get_logger().info("  cfe_mid: " + str(command_info[cmd]['cfe_mid']))
            self._node.get_logger().info("  cmd_code: " + str(command_info[cmd]['cmd_code']))
            self._cmd_map[command_info[cmd]['cfe_mid']] = cmd
            self._cmd_code_map[cmd] = command_info[cmd]['cmd_code']
        self._logger.info("command map is " + str(self._cmd_map))
        self._recv_buff_size = 4096

        self._running = True
        self._recv_thread = threading.Thread(target=self.receive_thread)

        self._logger.warn("starting thread to receive CFS command")
        self._recv_thread.start()
        self._current_value = {}

    def stop_thread(self):
        self._running = False
        self._recv_thread.join()

    def receive_thread(self):
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._logger.warn("binding to port " + str(self._port))
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
        packet_id = self.get_pkt_id(datagram)
        seq = self.get_seq_count(datagram)
        self._logger.warn("Sequence count = " + str(seq))
        if packet_id in self._cmd_map:
            self._logger.warn("Handling command message id " + packet_id)
            ros_name = self._cmd_map[packet_id]
            self._logger.info("Received packet for " + ros_name)
            MsgType = getattr(importlib.import_module(self._msg_pkg + ".msg"),
                              self._cmd_map[packet_id])
            msg = MsgType()
            self._juicer_interface.parse_packet(datagram, 0, self._cmd_map[packet_id], msg, self._msg_pkg)
            self._current_value[ros_name] = msg
            self._logger.info("Parsed cmd: " + str(msg))
        else:
            self._logger.warn("Don't know how to handle command message id " + packet_id)

    def get_latest_data(self, key):
        retval = None
        if key in self._current_value:
            retval = self._current_value[key]
        # else:
        #     self._logger.info("Can't find data for " + key)
        return retval

    @staticmethod
    def get_pkt_id(datagram):
        streamid = unpack(">H", datagram[:2])
        return hex(streamid[0])

    @staticmethod
    def get_seq_count(datagram):
        streamid = unpack(">H", datagram[2:4])
        return streamid[0] & 0x3FFF  # # sequence count mask
