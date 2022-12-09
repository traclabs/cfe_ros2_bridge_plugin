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

        self._node.declare_parameter('plugin_params.commandPort', 1234)
        self._command_port = self._node.get_parameter('plugin_params.commandPort'). \
            get_parameter_value().integer_value
        self._node.get_logger().info('commandPort: ' + str(self._command_port))

        self._node.declare_parameter('plugin_params.commandHost', '127.0.0.1')
        self._command_host = self._node.get_parameter('plugin_params.commandHost'). \
            get_parameter_value().string_value
        self._node.get_logger().info('commandHost: ' + str(self._command_host))

        self._msg_pkg = "cfe_msgs"

        self._telem_info = self._juicer_interface.get_telemetry_message_info()
        self._command_info = self._juicer_interface.get_command_message_info()

        command_params = ["structure", "cfe_mid", "cmd_code", "topic_name"]
        telemetry_params = ["cfe_mid", "topic_name"]
        self._cfe_config = ParseCFEConfig(self._node, command_params, telemetry_params)
        self._cfe_config.print_commands()
        self._cfe_config.print_telemetry()

        self._command_dict = self._cfe_config.get_command_dict()
        self._telemetry_dict = self._cfe_config.get_telemetry_dict()

        self._telem_receiver = TelemReceiver(self._node, self._msg_pkg, self._telemetry_port,
                                             self._telemetry_dict,
                                             self._juicer_interface)
        # self._cmd_receiver = CmdReceiver(self._node, self._msg_pkg, self._command_port,
        #                                      self._command_dict,
        #                                      self._juicer_interface)

        self._command_info = self._juicer_interface.reconcile_command_info(self._command_info, self._command_dict)
        for ci in self._command_info:
            key = ci.get_key()
            cmd_ids = self._command_dict[key]
            ch = CommandHandler(self._node, ci, self.command_callback, int(cmd_ids['cfe_mid'], 16), cmd_ids['cmd_code'])
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
        self._node.get_logger().debug('Handling cmd ' + key_name)
        cmd_ids = self._command_dict[key_name]
        self._node.get_logger().debug('Cmd ids: ' + str(cmd_ids))
        packet = self._juicer_interface.parse_command(command_info, message, cmd_ids['cfe_mid'], cmd_ids['cmd_code'])
        send_success = self.send_cmd_packet(packet, self._command_host, self._command_port)
        if send_success:
            self._node.get_logger().debug('Sent packet to cFE.')
        else:
            self._node.get_logger().warn('Failed to send packet to cFE!')

    def send_cmd_packet(self, packet, cmd_host, cmd_port):
        # send packet to cFE
        self._node.get_logger().debug('Got packet to send to cFE!')
        cmd_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        bytes_sent = cmd_sock.sendto(packet, (cmd_host, cmd_port))
        self._node.get_logger().debug('Sent ' + str(bytes_sent) + ' bytes out of ' + str(len(packet)))
        cmd_sock.close()
        return bytes_sent > 0

    def get_telemetry_message_info(self):
        return self._telem_info

    def get_command_message_info(self):
        return self._command_info

    def get_latest_data(self, key):
        return self._telem_receiver.get_latest_data(key)

    def create_ros_msgs(self, msg_dir):
        msg_list = []
        return msg_list

    def get_msg_package(self):
        return self._msg_pkg
