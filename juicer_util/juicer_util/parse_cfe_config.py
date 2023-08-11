"""
.. module:: cfe_ros2_bridge_plugin.juicer_util.parse_cfe_config
   :synopsis: Class that parses a config file for items related to the juicer utils

.. moduleauthor:: Tod Milam

"""

from rcl_interfaces.msg import ParameterDescriptor
from rclpy.parameter import Parameter


class ParseCFEConfig():
    """This class parses and holds the items related to the juicer_utils from a configuration file.

        Attributes
        ----------
        node : rosnode
            the ROS2 node
        command_dict : dict
            the command object dictionary
        telemetry_dict : dict
            the telemetry object dictionary

        Methods
        -------
    print_commands():
        Helper method to output the list of commands to log.
    print_telemetry():
        Helper method to output the list of telemetry to log.
    parse_parameter(p):
        Parse a parameter for the command/telemetry entry.
    get_command_dict():
        Return the command object dictionary
    get_telemetry_dict():
        Return the telemetry object dictionary
    """

    def __init__(self, node, command_params=[], telemetry_params=[]):
        '''
        Initializes the attributes for this object.

        Args:
            node (rosnode): The ROS2 node
            command_params (list): The list of parameters to read for each command object
            telemetry_params (list): The list of parameters to read for each telemetry object
        '''
        self._node = node
        self._command_dict = {}
        self._telemetry_dict = {}
        # self._telemetry_port = 0

        self._node.get_logger().info('parsing cFE config file...')

        self._node.declare_parameters(
            namespace="",
            parameters=[
                ('plugin_params.commands', [], ParameterDescriptor(name='plugin_params.commands',
                                                                   dynamic_typing=True)),
                ('plugin_params.telemetry', [], ParameterDescriptor(name='plugin_params.telemetry',
                                                                    dynamic_typing=True))
                ]
        )

        commands = self._node.get_parameter('plugin_params.commands').get_parameter_value(). \
            string_array_value
        self._node.get_logger().debug('commands: ')
        for cmd in commands:
            self._node.get_logger().debug('  ' + cmd)
            params = {}
            for cp in command_params:
                command_param = "plugin_params.command_data." + cmd + "." + cp
                self._node.declare_parameters(
                    namespace="",
                    parameters=[
                        (command_param, [], ParameterDescriptor(name=command_param,
                                                                dynamic_typing=True))
                    ]
                )
                c = self.parse_parameter(self._node.get_parameter(command_param))
                self._node.get_logger().debug('    ' + cp + ": " + str(c))
                params[cp] = c
            self._command_dict[cmd] = params

        telemetry = self._node.get_parameter('plugin_params.telemetry').get_parameter_value(). \
            string_array_value
        self._node.get_logger().debug('telemetry: ')
        for tlm in telemetry:
            self._node.get_logger().debug('  ' + tlm)
            params = {}
            for tp in telemetry_params:
                telemetry_param = "plugin_params.telemetry_data." + tlm + "." + tp
                self._node.declare_parameters(
                    namespace="",
                    parameters=[
                        (telemetry_param, [], ParameterDescriptor(name=telemetry_param,
                                                                  dynamic_typing=True))
                    ]
                )
                t = self.parse_parameter(self._node.get_parameter(telemetry_param))
                self._node.get_logger().debug('    ' + tp + ": " + str(t))
                params[tp] = t
            self._telemetry_dict[tlm] = params

    def print_commands(self):
        '''
        Helper method to output the list of commands to log.
        '''
        self._node.get_logger().info('commands: ')
        for key in self._command_dict:
            self._node.get_logger().info('  ' + key + ":")
            for pk, pv in self._command_dict[key].items():
                self._node.get_logger().info('    ' + pk + ": " + str(pv))

    def print_telemetry(self):
        '''
        Helper method to output the list of telemetry to log.
        '''
        self._node.get_logger().info('telemetry: ')
        for key in self._telemetry_dict:
            self._node.get_logger().info('  ' + key + ":")
            for pk, pv in self._telemetry_dict[key].items():
                self._node.get_logger().info('    ' + pk + ": " + str(pv))

    def parse_parameter(self, p):
        '''
        Parse a parameter for the command/telemetry entry.

        Args:
            p: The parameter to parse

        Returns:
            value:  The value of the parameter
        '''
        if p.type_ == Parameter.Type.STRING:
            return p.get_parameter_value().string_value
        elif p.type_ == Parameter.Type.BOOL:
            return p.get_parameter_value().bool_value
        elif p.type_ == Parameter.Type.DOUBLE:
            return p.get_parameter_value().double_value
        elif p.type_ == Parameter.Type.INTEGER:
            return p.get_parameter_value().integer_value
        elif p.type_ == Parameter.Type.INTEGER_ARRAY:
            return p.get_parameter_value().integer_array_value
        elif p.type_ == Parameter.Type.BYTE_ARRAY:
            return p.get_parameter_value().byte_array_value
        elif p.type_ == Parameter.Type.STRING_ARRAY:
            return p.get_parameter_value().string_array_value
        elif p.type_ == Parameter.Type.DOUBLE_ARRAY:
            return p.get_parameter_value().double_array_value
        return 0

    def get_command_dict(self):
        '''
        Return the command object dictionary

        Returns:
            command_dict (dict): The dictionary of command objects
        '''
        return self._command_dict

    def get_telemetry_dict(self):
        '''
        Return the telemetry object dictionary

        Returns:
            telemetry_dict (dict): The dictionary of telemetry objects
        '''
        return self._telemetry_dict
