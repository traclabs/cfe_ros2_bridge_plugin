from fsw_ros2_bridge.fsw_plugin_interface import FSWPluginInterface

# from fsw_ros2_bridge.telem_info import TelemInfo
# from fsw_ros2_bridge.command_info import CommandInfo

from cfe_plugin.juicer_interface import JuicerInterface

from cfe_plugin.telem_receiver import TelemReceiver
from cfe_plugin.parse_cfe_config import ParseCFEConfig

# from pathlib import Path


class FSWPlugin(FSWPluginInterface):

    def __init__(self, node):

        self._node = node
        self._node.get_logger().info("Setting up cFE plugin")

        self._juicer_interface = JuicerInterface(self._node)
        # self.RoutingService = None

        self._node.declare_parameter('plugin_params.telemetryPort', 0)
        self._telemetry_port = self._node.get_parameter('plugin_params.telemetryPort').\
            get_parameter_value().integer_value
        self._node.get_logger().info('telemetryPort: ' + str(self._telemetry_port))
        # self._node.declare_parameter('cfs_root', '/home/swhart/code/cFS')
        # self.cfs_root = self._node.get_parameter('/home/swhart/code/cFS').get_parameter_value().\
        #   string_value

        # self._node.get_logger().info("  using cfs_root: " + self.cfs_root)

        # self.ROOTDIR = self.cfs_root + "/tools/cFS-GroundSystem/Subsystems"
        # self.tlmDefFile = f"{self.ROOTDIR}/tlmGUI/telemetry-pages.txt"
        # self.cmdDefFile = f"{self.ROOTDIR}/cmdGui/command-pages.txt"

        self._msg_pkg = "cfe_msgs"
        # self.telem_pages_info = TelemPagesInfo(self.tlmDefFile)
        # self.cmd_pages_info = CmdPagesInfo(self.cmdDefFile)

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
        # for i in range(self.telem_pages_info.getTelemMapSize()):
        #     key = self.telem_pages_info.getTelemDesc(i)
        #     # if key != "ES HK Tlm": continue
        #     deffile = self.telem_pages_info.getTelemDefFile(i)
        #     msgType = self.telem_pages_info.getROSMsgName(i)
        #     if deffile == "null":
        #         continue
        #     self._recv_map[key] = CFSTelemReciever(self.telem_pages_info, key,
        #                                            self.telem_pages_info.getTelemAppid(i),
        #                                            f"{self.ROOTDIR}/tlmGUI/", deffile)
        #     topicName = Path(deffile).stem.replace("-", "_")
        #     t = TelemInfo(key, msgType, topicName)
        #     self._telem_info.append(t)

        self._telem_receiver = TelemReceiver(self._node, self._msg_pkg, self._telemetry_port,
                                             self._telemetry_dict,
                                             self._juicer_interface.getMsg_list())
        # self.broad_map = {}
        # for i in range(self.cmd_pages_info.getCmdMapSize()):
        #     key = self.cmd_pages_info.getCmdDesc(i)
        #     deffile = self.cmd_pages_info.getCmdDefFile(i)
        #     msgType = self.cmd_pages_info.getROSMsgName(i)
        #     if deffile == "":
        #         continue
        #     topicName = Path(deffile).stem.replace("-", "_").lower()
        #     topicName = topicName.replace("__", "_")
        #     self.broad_map[key] = CFSCommandBroadcaster(self.cmd_pages_info, key,
        #                                                 self.cmd_pages_info.getCmdAppid(i),
        #                                                 f"{self.ROOTDIR}/cmdGui/",
        #                                                 deffile, topicName)
        #     c = CommandInfo(key, msgType, topicName, self.broad_map[key].processCallback)
        #     # c.print()
        #     self._command_info.append(c)

        # self.initRoutingService()

        # def initRoutingService(self):
        #     self.RoutingService = RoutingService()
        #     self.RoutingService.start()

    def get_telemetry_message_info(self):
        return self._telem_info

    def get_command_message_info(self):
        return self._command_info

    def get_latest_data(self, key):
        try:
            return self._recv_map[key].get_latest_data()
        except (KeyError):
            self._node.get_logger().error("No key " + key + " in receive map")

    def create_ros_msgs(self, msg_dir):
        msg_list = []
        return msg_list

    def get_msg_package(self):
        return self._msg_pkg
