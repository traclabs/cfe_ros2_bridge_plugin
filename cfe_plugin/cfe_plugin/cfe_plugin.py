"""
.. module:: cfe_ros2_bridge_plugin.cfe_plugin.cfe_plugin
   :synopsis: Class that serves as the main class for the cFE plugin

.. moduleauthor:: Tod Milam

"""

import os
import socket
from fsw_ros2_bridge.fsw_plugin_interface import FSWPluginInterface

from juicer_util.juicer_interface import JuicerInterface
from juicer_util.parse_cfe_config import ParseCFEConfig

from cfe_plugin.telem_receiver import TelemReceiver
from cfe_plugin.command_handler import CommandHandler

from rcl_interfaces.msg import SetParametersResult

from ament_index_python.packages import get_package_share_directory


class FSWPlugin(FSWPluginInterface):
    """This class is the main class of the cFE GroundSystem Plugin.

    Attributes
    ----------
    node : rosnode
        the ROS2 node
    juicer_interface : JuicerInterface
        the interface to the juicer database
    telemetry_port : int
        the port to receive telemetry from
    command_host : str
        the host name to send commands
    command_ports : dict
        the list of ports that commands will be sent on
    msg_pkg : str
        the name of the ROS2 package containing the message structures
    telem_info : list
        the list of telemetry items to listen for
    command_info : list
        the list of command items to listen for
    cfe_config : ParseCFEConfig
        the configuration values from the configuration file(s)
    command_dict : dict
        the list of commands from the configuration file(s)
    telemetry_dict : dict
        the list of telemetry from the configuration file(s)
    telem_receivers : list
        the list of TelemReceiver objects

    Methods
    -------
    parameters_callback(params):
        Method called when a parameter has been changed.
    command_callback(command_info, message):
        Method called when a command is received.
    send_cmd_packet(packet):
        Sends the command packet to cFE.
    get_telemetry_message_info():
        Returns the list of telemetry info objects.
    get_command_message_info():
        Returns the list of command info objects.
    get_latest_data(key):
        Return the latest value for the specified key.
    create_ros_msgs(msg_dir):
        Unused method required by interface.
    get_msg_package():
        Return the package where the ROS2 messages are found.
    """

    def __init__(self, node):
        '''
        Initializes the attributes and set up command and telemetry listeners.

        Args:
            node (rosnode): The ROS2 node
        '''
        self._node = node
        self._node.get_logger().info("Setting up cFE plugin")

        resource_path = get_package_share_directory("cfe_plugin") + "/resource/"
        self._juicer_interface = JuicerInterface(self._node, resource_path)

        self._node.declare_parameter('plugin_params.udp_receive_port', 1235)
        self._telemetry_port = self._node.get_parameter('plugin_params.udp_receive_port'). \
            get_parameter_value().integer_value
        self._node.get_logger().info('udp_receive_port: ' + str(self._telemetry_port))

        self._node.declare_parameter('plugin_params.udp_send_port', 1234)
        self._command_port = os.environ.get("FSW_CMD_PORT")
        if not self._command_port:
            self._command_port = self._node.get_parameter('plugin_params.udp_send_port'). \
                get_parameter_value().integer_value
        else:
            self._command_port = int(self._command_port)
        self._node.get_logger().info('udp_send_port: ' + str(self._command_port))

        # Bind address
        self._node.declare_parameter('plugin_params.udp_telemetry_ip', '0.0.0.0')
        self._telemetry_ip = self._node.get_parameter('plugin_params.udp_telemetry_ip'). \
             get_parameter_value().string_value
        self._node.get_logger().info('udp_telemetry_ip: ' + str(self._telemetry_ip))

        # Transmit Address
        self._node.declare_parameter('plugin_params.udp_command_ip', '127.0.0.1')
        self._command_ip = os.environ.get("FSW_IP")
        if not self._command_ip:
            self._command_ip = self._node.get_parameter('plugin_params.udp_command_ip'). \
                get_parameter_value().string_value
        self._node.get_logger().info('udp_command_ip: ' + str(self._command_ip))

        # set up socket
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.bind((self._telemetry_ip, self._telemetry_port))
        self._sock.setblocking(False)

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
        telem_receiver = TelemReceiver(self._node,
                                       self._msg_pkg,
                                       self._sock,
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

        self._node.add_on_set_parameters_callback(self.parameters_callback)

    def parameters_callback(self, params):
        '''
        Method called when a parameter has been changed.

        Args:
            params (list): The list of new parameter values

        Returns:
            result (bool): If set was successful
        '''
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
        '''
        Method called when a command is received.

        Args:
            command_info (JuicerCommandEntry): The command info of the message received
            message (): The command message values
        '''
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
        '''
        Sends the command packet to cFE.

            Parameters:
                    packet (bytearray): The data to send
            Returns:
                    success (bool): If the packet was sent successfully
        '''
        # send packet to cFE
        self._node.get_logger().info(f"Got packet to send to cFE! ({self._command_ip}, {self._command_port})")

        send_worked = False
        try:
            self._sock.sendto(packet, (self._command_ip, self._command_port))
            send_worked = True
            self._node.get_logger().debug('Sent command data.')
        except OSError as err:
            # TODO: assume socket closed and reopen
            self._node.get_logger().warn('socket error: ' + str(err))
        return send_worked

    def get_telemetry_message_info(self):
        '''
        Returns the list of telemetry info objects.

        Returns:
            telem_info (list): List of TelemetryInfo objects
        '''
        return self._telem_info

    def get_command_message_info(self):
        '''
        Returns the list of command info objects.

        Returns:
            command_info (list): List of CommandInfo objects
        '''
        return self._command_info

    def get_buffered_data(self, key, clear=True):
        '''
        Return the latest value for the specified key.

            Parameters:
                    key (str): The ROS2 name of the telemetry wanted
                    clear (bool): If queue should be cleared

        Returns:
            latest_data (): The value of the specified key
        '''
        data = None
        for telem_receiver in self._telem_receivers:
            if data == None:
                data = telem_receiver.get_buffered_data(key, clear)
        return data

    def create_ros_msgs(self, msg_dir):
        '''
        Unused method required by interface.
        '''
        msg_list = []
        return msg_list

    def get_msg_package(self):
        '''
        Return the package where the ROS2 messages are found.

        Returns:
            msg_pkg (str): Package name where ROS2 message are located
        '''
        return self._msg_pkg
