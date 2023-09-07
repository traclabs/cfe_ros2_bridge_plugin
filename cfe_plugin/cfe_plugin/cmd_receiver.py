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
            self._node.get_logger().debug("type: " + str(cmd))
            self._node.get_logger().debug("  cfe_mid: " + str(command_info[cmd]['cfe_mid']))
            self._node.get_logger().debug("  cmd_code: " + str(command_info[cmd]['cmd_code']))
            self._cmd_map[command_info[cmd]['cfe_mid']] = cmd
            self._cmd_code_map[cmd] = command_info[cmd]['cmd_code']
        self._logger.debug("command map is " + str(self._cmd_map))
        self._recv_buff_size = 4096


        self._timer_period = 0.05
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.bind(("", self._port))
        self._sock.setblocking(False)

        self._logger.info("starting thread to receive CFS command")
        self._running = True
        self._recv_timer = self._node.create_timer(self._timer_period, self.receive_callback)

        self._latest_values = {}

    def stop_thread(self):
        self._running = False
        self._recv_timer.shutdown()

    def receive_callback(self):
        if self._running:
            try:
                self._logger.info("cmd_reciever.callback() time since last call: " + str(self._recv_timer.time_since_last_call() * 1e-9))

                # # receive message
                datagram, host = self._sock.recvfrom(self._recv_buff_size)

                # ignore data if not long enough (doesn't contain header)
                if len(datagram) < 6:
                    return

                self.handle_packet(datagram)

            except socket.error:
                # self._logger.warn("Error receiving command data.")
                pass

    def handle_packet(self, datagram):
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
