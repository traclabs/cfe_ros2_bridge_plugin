#!/usr/bin/env python3

from rcl_interfaces.msg import ParameterDescriptor
from rclpy.parameter import Parameter


class ParseCFEConfig():

    def __init__(self, node, command_params=[], telemetry_params=[]):

        self._node = node
        self._command_dict = {}
        self._telemetry_dict = {}

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
        self._node.get_logger().info('commands: ')
        for key in self._command_dict:
            self._node.get_logger().info('  ' + key + ":")
            for pk, pv in self._command_dict[key].items():
                self._node.get_logger().info('    ' + pk + ": " + str(pv))

    def print_telemetry(self):
        self._node.get_logger().info('telemetry: ')
        for key in self._telemetry_dict:
            self._node.get_logger().info('  ' + key + ":")
            for pk, pv in self._telemetry_dict[key].items():
                self._node.get_logger().info('    ' + pk + ": " + str(pv))

    def parse_parameter(self, p):
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
        return self._command_dict

    def get_telemetry_dict(self):
        return self._telemetry_dict
