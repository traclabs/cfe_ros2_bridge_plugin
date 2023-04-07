import socket
from fsw_ros2_bridge.fsw_plugin_interface import FSWPluginInterface

from juicer_util.juicer_interface import JuicerInterface
from juicer_util.parse_cfe_config import ParseCFEConfig

from cfe_plugin.telem_receiver import TelemReceiver
from cfe_plugin.cmd_receiver import CmdReceiver
from cfe_plugin.command_handler import CommandHandler

from rcl_interfaces.msg import SetParametersResult

from ament_index_python.packages import get_package_share_directory


class FSWPlugin(FSWPluginInterface):

    def __init__(self, node):

        self._node = node
        self._node.get_logger().info("Setting up cFE plugin")

        resource_path = get_package_share_directory("cfe_plugin") + "/resource/"
        self._juicer_interface = JuicerInterface(self._node, resource_path)

        self._node.declare_parameter('plugin_params.telemetryPort', 0)
        self._telemetry_port = self._node.get_parameter('plugin_params.telemetryPort'). \
            get_parameter_value().integer_value
        self._node.get_logger().info('telemetryPort: ' + str(self._telemetry_port))

        self._node.declare_parameter('plugin_params.commandHost', '127.0.0.1')
        self._command_host = self._node.get_parameter('plugin_params.commandHost'). \
            get_parameter_value().string_value
        self._node.get_logger().info('commandHost: ' + str(self._command_host))

        self._command_ports = {}

        self._msg_pkg = "cfe_msgs"

        self._telem_info = self._juicer_interface.get_telemetry_message_info()
        self._command_info = self._juicer_interface.get_command_message_info()

        command_params = ["structure", "cfe_mid", "cmd_code", "topic_name", "port"]
        telemetry_params = ["structure", "cfe_mid", "topic_name", "port"]
        self._cfe_config = ParseCFEConfig(self._node, command_params, telemetry_params)
        self._cfe_config.print_commands()
        self._cfe_config.print_telemetry()

        self._command_dict = self._cfe_config.get_command_dict()
        self._telemetry_dict = self._cfe_config.get_telemetry_dict()
        tlm_ports = []
        for tlm in self._telemetry_dict:
            port = self._telemetry_dict[tlm]['port']
            if port not in tlm_ports:
                tlm_ports.append(port)
        self._node.get_logger().info("Telemetry ports: " + str(tlm_ports))

        self._telem_info = self._juicer_interface.reconcile_telem_info(self._telem_info, self._telemetry_dict)
        self._telem_receivers = []
        for port in tlm_ports:
            telem_receiver = TelemReceiver(self._node, self._msg_pkg, port,
                                           self._telemetry_dict,
                                           self._juicer_interface)
            self._telem_receivers.append(telem_receiver)
        # self._cmd_receiver = CmdReceiver(self._node, self._msg_pkg, self._command_port,
        #                                      self._command_dict,
        #                                      self._juicer_interface)

        self._command_info = self._juicer_interface.reconcile_command_info(self._command_info, self._command_dict)
        symbol_name_map = self._juicer_interface.get_symbol_ros_name_map()
        for ci in self._command_info:
            key = ci.get_key()
            cmd_ids = self._command_dict[key]
            msg_size = symbol_name_map[ci.get_msg_type()].get_size()
            ch = CommandHandler(self._node, ci, self.command_callback, int(cmd_ids['cfe_mid'], 16), cmd_ids['cmd_code'], msg_size)
            ci.set_callback_func(ch.process_callback)

        self._node.add_on_set_parameters_callback(self.parameters_callback)

    def parameters_callback(self, params):
        self._node.get_logger().warn("param callback!")
        for param in params:
            if param.name == "plugin_params.telemetryPort":
                self._telemetry_port = param.value
                self._node.get_logger().info('Got a telemetryPort update: '
                                             + str(self._telemetry_port))
        return SetParametersResult(successful=True)

    def command_callback(self, command_info, message):
        key_name = command_info.get_key()
        self._node.get_logger().info('Handling cmd ' + key_name)
        cmd_ids = self._command_dict[key_name]
        self._node.get_logger().info('Cmd ids: ' + str(cmd_ids))
        packet = self._juicer_interface.parse_command(command_info, message, cmd_ids['cfe_mid'], cmd_ids['cmd_code'])

        if (key_name == 'CPU1RobotSimJointCmdt'):
            packet[0] = 0x18
            packet[1] = 0x17
            packet[6] = cmd_ids['cmd_code']

        send_success = self.send_cmd_packet(packet, self._command_host, cmd_ids['port'])

        if send_success:
            self._node.get_logger().info('Sent packet of size ' + str(len(packet)) + ' to cFE.\n' + str(packet))
        else:
            self._node.get_logger().warn('Failed to send packet to cFE!')

    def send_cmd_packet(self, packet, cmd_host, cmd_port):
        # send packet to cFE
        self._node.get_logger().info('Got packet to send to cFE!')
        if cmd_port in self._command_ports:
            cmd_sock = self._command_ports.get(cmd_port)
        else:
            cmd_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self._command_ports[cmd_port] = cmd_sock
            cmd_sock.connect((cmd_host, cmd_port))
        send_worked = False
        try:
            cmd_sock.sendall(packet)
            send_worked = True
            self._node.get_logger().debug('Sent command data.')
        except OSError as err:
            # TODO: assume socket closed and reopen
            self._node.get_logger().warn('socket error: ' + err)
        return send_worked

    def get_telemetry_message_info(self):
        return self._telem_info

    def get_command_message_info(self):
        return self._command_info

    def get_latest_data(self, key):
        data = None
        for telem_receiver in self._telem_receivers:
            if data == None:
                data = telem_receiver.get_latest_data(key)

        return data

    def create_ros_msgs(self, msg_dir):
        msg_list = []
        return msg_list

    def get_msg_package(self):
        return self._msg_pkg
