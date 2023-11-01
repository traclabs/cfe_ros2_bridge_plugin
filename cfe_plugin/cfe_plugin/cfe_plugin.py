import socket
from fsw_ros2_bridge.fsw_plugin_interface import FSWPluginInterface

from juicer_util.juicer_interface import JuicerInterface
from juicer_util.parse_cfe_config import ParseCFEConfig

from cfe_plugin.telem_receiver import TelemReceiver
from cfe_plugin.command_handler import CommandHandler

from rcl_interfaces.msg import SetParametersResult

from ament_index_python.packages import get_package_share_directory


class FSWPlugin(FSWPluginInterface):

    def __init__(self, node):

        self._node = node
        self._node.get_logger().info("Setting up cFE plugin")

        resource_path = get_package_share_directory("cfe_plugin") + "/resource/"
        self._juicer_interface = JuicerInterface(self._node, resource_path)

        self._node.declare_parameter('plugin_params.udp_receive_port', 1235)
        self._telemetry_port = self._node.get_parameter('plugin_params.udp_receive_port'). \
            get_parameter_value().integer_value
        self._node.get_logger().info('udp_receive_port: ' + str(self._telemetry_port))

        self._node.declare_parameter('plugin_params.udp_send_port', 1234)
        self._command_port = self._node.get_parameter('plugin_params.udp_send_port'). \
            get_parameter_value().integer_value
        self._node.get_logger().info('udp_send_port: ' + str(self._command_port))

        self._node.declare_parameter('plugin_params.udp_receive_ip', '127.0.0.1')
        self._receive_ip = self._node.get_parameter('plugin_params.udp_receive_ip'). \
             get_parameter_value().string_value
        self._node.get_logger().info('udp_receive_ip: ' + str(self._receive_ip))

        self._node.declare_parameter('plugin_params.udp_send_ip', '127.0.0.1')
        self._send_ip = self._node.get_parameter('plugin_params.udp_send_ip'). \
             get_parameter_value().string_value
        self._node.get_logger().info('udp_send_ip: ' + str(self._send_ip))

        self._node.get_logger().info("Telemetry port: " + str(self._telemetry_port))
        self._node.get_logger().info("Command port: " + str(self._command_port))

        # msg info
        self._msg_pkg = "cfe_msgs"
        self._telem_info = self._juicer_interface.get_telemetry_message_info()
        self._command_info = self._juicer_interface.get_command_message_info()

        command_params = ["structure", "cfe_mid", "cmd_code", "topic_name"]
        telemetry_params = ["structure", "cfe_mid", "topic_name"]
        self._cfe_config = ParseCFEConfig(self._node, command_params, telemetry_params)
        self._cfe_config.print_commands()
        self._cfe_config.print_telemetry()

        self._command_dict = self._cfe_config.get_command_dict()
        self._telemetry_dict = self._cfe_config.get_telemetry_dict()

        # set up telemetry receivers
        self._telem_info = self._juicer_interface.reconcile_telem_info(self._telem_info, self._telemetry_dict)
        self._telem_receivers = []
        telem_receiver = TelemReceiver(self._node, self._msg_pkg,
                                       self._receive_ip,
                                       self._telemetry_port,
                                       self._telemetry_dict,
                                       self._juicer_interface)
        self._telem_receivers.append(telem_receiver)

        # set up command broadcasters
        self._command_info = self._juicer_interface.reconcile_command_info(self._command_info, self._command_dict)
        symbol_name_map = self._juicer_interface.get_symbol_ros_name_map()
        for ci in self._command_info:
            key = ci.get_key()
            cmd_ids = self._command_dict[key]

            msg_type = ci.get_msg_type()
            if not msg_type:
                # Special case: Default handler for binary command payload (with cfg defined MID + FC)
                # We specify size of 0 to indicate dynamically sized message
                msg_size = 0
                self._node.get_logger().warn('cmd with msg_size=0 (dynamic)')
            else:
                msg_size = symbol_name_map[ci.get_msg_type()].get_size()
            ch = CommandHandler(self._node, ci, self.command_callback, int(cmd_ids['cfe_mid'], 16), cmd_ids['cmd_code'], msg_size)
            ci.set_callback_func(ch.process_callback)

        self._command_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._command_socket.connect((self._send_ip, self._command_port))

        self._node.add_on_set_parameters_callback(self.parameters_callback)

    def parameters_callback(self, params):
        self._node.get_logger().warn("param callback!")
        for param in params:
            if param.name == "plugin_params.udp_receive_port":
                self._telemetry_port = param.value
                self._node.get_logger().info('Got a udp_receive_port update: '
                                             + str(self._telemetry_port))
            if param.name == "plugin_params.udp_send_port":
                self._command_port = param.value
                self._node.get_logger().info('Got a udp_send_port update: '
                                             + str(self._command_port))
        return SetParametersResult(successful=True)

    def command_callback(self, command_info, message):
        key_name = command_info.get_key()
        self._node.get_logger().info('Handling cmd ' + key_name)
        cmd_ids = self._command_dict[key_name]
        self._node.get_logger().info('Cmd ids: ' + str(cmd_ids))
        packet = self._juicer_interface.parse_command(command_info, message, cmd_ids['cfe_mid'], cmd_ids['cmd_code'])

        send_success = self.send_cmd_packet(packet)

        if send_success:
            self._node.get_logger().debug('Sent packet of size ' + str(len(packet)) + ' to cFE.\n' + str(packet))
            self._node.get_logger().info('Sent packet of size ' + str(len(packet)) + ' to cFE.')
        else:
            self._node.get_logger().warn('Failed to send packet to cFE!')

    def send_cmd_packet(self, packet):
        # send packet to cFE
        self._node.get_logger().info('Got packet to send to cFE!')
        send_worked = False
        try:
            self._command_socket.sendall(packet)
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

    def get_buffered_data(self, key, clear=True):
        data = None
        for telem_receiver in self._telem_receivers:
            if data == None:
                data = telem_receiver.get_buffered_data(key, clear)
        return data

    def create_ros_msgs(self, msg_dir):
        msg_list = []
        return msg_list

    def get_msg_package(self):
        return self._msg_pkg
