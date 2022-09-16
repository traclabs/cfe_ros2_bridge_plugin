from fsw_ros2_bridge.fsw_plugin_interface import FSWPluginInterface

from juicer_util.juicer_interface import JuicerInterface

from cfe_plugin.telem_receiver import TelemReceiver
from cfe_plugin.parse_cfe_config import ParseCFEConfig


class FSWPlugin(FSWPluginInterface):

    def __init__(self, node):

        self._node = node
        self._node.get_logger().info("Setting up cFE plugin")

        self._juicer_interface = JuicerInterface(self._node)

        self._node.declare_parameter('plugin_params.telemetryPort', 0)
        self._telemetry_port = self._node.get_parameter('plugin_params.telemetryPort'). \
            get_parameter_value().integer_value
        self._node.get_logger().info('telemetryPort: ' + str(self._telemetry_port))

        self._msg_pkg = "cfe_msgs"

        self._telem_info = self._juicer_interface.get_telemetry_message_info()
        self._command_info = self._juicer_interface.get_command_message_info()

        command_params = ["cfe_mid", "cmd_code"]
        telemetry_params = ["cfe_mid", "topic_name"]
        self._cfe_config = ParseCFEConfig(self._node, command_params, telemetry_params)
        self._cfe_config.print_commands()
        self._cfe_config.print_telemetry()

        self._command_dict = self._cfe_config.get_command_dict()
        self._telemetry_dict = self._cfe_config.get_telemetry_dict()

        self._recv_map = {}

        self._telem_receiver = TelemReceiver(self._node, self._msg_pkg, self._telemetry_port,
                                             self._telemetry_dict,
                                             self._juicer_interface)

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
