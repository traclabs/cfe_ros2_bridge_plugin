#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from rcl_interfaces.msg import ParameterDescriptor
from rclpy.parameter import Parameter

class ParseCFEConfig(Node):

  def __init__(self, command_params=[], telemetry_params=[]):
    super().__init__('parse_cfe_config')

    self.command_dict = {}
    self.telemetry_dict = {}
    self.telemetry_port = 0

    self.get_logger().info('parsing cFE config file...')

    self.declare_parameters(
      namespace="",
      parameters=[
        ('telemetryPort', [], ParameterDescriptor(name='telemetryPort', dynamic_typing=True)),
        ('commands', [], ParameterDescriptor(name='commands', dynamic_typing=True)),
        ('telemetry', [], ParameterDescriptor(name='telemetry', dynamic_typing=True))
      ]
    )

    self.telemetry_port = self.get_parameter('telemetryPort').get_parameter_value().integer_value
    self.get_logger().debug('telemetryPort: ' + str(self.telemetry_port))

    commands = self.get_parameter('commands').get_parameter_value().string_array_value
    self.get_logger().debug('commands: ')
    for cmd in commands:
      self.get_logger().debug('  ' + cmd)
      params = {}
      for cp in command_params:
        command_param = cmd + "." + cp
        self.declare_parameters(
          namespace="",
          parameters=[
            (command_param, [], ParameterDescriptor(name=command_param, dynamic_typing=True))
          ]
        )
        c = self.parseParameter(self.get_parameter(command_param))
        self.get_logger().debug('    ' + cp + ": " + str(c))
        params[cp] = c
      self.command_dict[cmd] = params

    telemetry = self.get_parameter('telemetry').get_parameter_value().string_array_value
    self.get_logger().debug('telemetry: ')
    for tlm in telemetry:
      self.get_logger().debug('  ' + tlm)
      params = {}
      for tp in telemetry_params:
        telemetry_param = tlm + "." + tp
        self.declare_parameters(
          namespace="",
          parameters=[
            (telemetry_param, [], ParameterDescriptor(name=telemetry_param, dynamic_typing=True))
          ]
        )
        t = self.parseParameter(self.get_parameter(telemetry_param))
        self.get_logger().debug('    ' + tp + ": " + str(t))
        params[tp] = t
      self.telemetry_dict[tlm] = params

    self.printCommands()
    self.printTelemetry()

  def printCommands(self):
    self.get_logger().info('commands: ')
    for key in self.command_dict:
      self.get_logger().info('  ' + key + ":")
      for pk, pv in self.command_dict[key].items():
        self.get_logger().info('    ' + pk + ": " + str(pv))

  def printTelemetry(self):
    self.get_logger().info('telemetry port: ' + str(self.telemetry_port))
    self.get_logger().info('telemetry: ')
    for key in self.telemetry_dict:
      self.get_logger().info('  ' + key + ":")
      for pk, pv in self.telemetry_dict[key].items():
        self.get_logger().info('    ' + pk + ": " + str(pv))

  def parseParameter(self, p) :
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

  def getCommandDict(self) :
    return self.command_dict

  def getTelemetryDict(self) :
    return self.telemetry_dict

  def getTelemetryPort(self) :
    return self.telemetry_port


def main(args=None):
    rclpy.init(args=args)

    command_params = ["cfeId", "cmdCode"]
    telemetry_params = ["cfeId"]

    parse_config = ParseCFEConfig(command_params, telemetry_params)

    rclpy.spin(parse_config)

    parse_config.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()