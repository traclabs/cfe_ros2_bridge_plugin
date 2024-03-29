"""
.. module:: cfe_ros2_bridge_plugin.cfe_msg_converter.cfe_msg_converter
   :synopsis: Tool to create ROS2 messages from juicer sql database

.. moduleauthor:: Tod Milam

"""
import rclpy
from rclpy.node import Node
import os

from ament_index_python.packages import get_package_share_directory

from juicer_util.juicer_interface import JuicerInterface
from juicer_util.juicer_symbols import field_byte_order


class CfeMsgConverter(Node):
    """This class converts a juicer-generated sql database into ROS2 message files.

    Attributes
    ----------
    cfs_root : str
        the directory containing the cFS code
    cfs_msgs_dir :
        the top level directory where the ROS2 messages will be stored
    msgs_src_dir :
        the source directory where the ROS2 messages will be written
    resource_path :
        the directory where resource files will be found
    juicer_interface :
        the interface to the juicer sql database
    symbol_name_map :
        the list of data structures from the juicer database to be converted to ROS2 messages
    msgs_list :
        the list of ROS2 messages to write out

    Methods
    -------
    create_messages(msgs_dir):
        Creates the message files.
    output_msg_file(symbol, msgs_dir):
        Writes a message file.
    get_location():
        Creates and returns the directory to write the messages file.
    """
    def __init__(self):
        '''
        Initializes the attributes and calls other methods to generate the messages.
        '''
        super().__init__('cfe_msg_converter')

        self._cfs_msgs_dir = get_package_share_directory("cfe_msgs")

        self._msgs_src_dir = self.get_location()

        self.get_logger().info("cfe_msgs_dir: " + self._cfs_msgs_dir)

        pkg_name = "cfe_msg_converter"
        self._resource_path = get_package_share_directory(pkg_name) + "/resource"
        self.get_logger().info("resource_path: " + self._resource_path)

        resource_path = get_package_share_directory("cfe_msg_converter") + "/resource/"
        self._juicer_interface = JuicerInterface(self, resource_path)
        self._symbol_name_map = self._juicer_interface.get_symbol_name_map()
        self._msgs_list = self.create_messages(self._msgs_src_dir)

    def create_messages(self, msgs_dir):
        '''
        Creates the message files.

        Args:
            msgs_dir (str): The directory to write the messages files in

        Returns:
            msg_list (list): The list of ROS2 message names written to file
        '''
        for key in self._symbol_name_map.keys():
            if self._symbol_name_map[key].get_should_output():
                self.output_msg_file(self._symbol_name_map[key], msgs_dir)

        msg_list = []
        for key in self._symbol_name_map.keys():
            symbol = self._symbol_name_map[key]
            if len(symbol.get_fields()) > 0 and symbol.get_should_output():
                mn = symbol.get_ros_name()
                if mn[0].isupper():
                    msg_list.append(mn)
        return msg_list

    def output_msg_file(self, symbol, msgs_dir):
        '''
        Writes a message file.

        Args:
            symbol (JuicerSymbolEntry): The symbol to write
            msgs_dir (str): The directory the file will be written in
        '''
        if len(symbol.get_fields()) > 0:
            v_names = {}
            mn = symbol.get_ros_name()
            fn = msgs_dir + "/msg/" + mn + ".msg"

            try:
                self.get_logger().info("writing: " + mn + ".msg")
                f = open(fn, "w")
                f.write("# cFE NAME: " + symbol.get_name() + "\n")
                f.write("# ROS topic: " + symbol.get_ros_topic() + "\n")
                if symbol.get_is_command():
                    f.write("# Command message" + "\n")
                    f.write("std_msgs/Header header" + "\n")
                if symbol.get_is_telemetry():
                    f.write("# Telemetry message" + "\n")
                    f.write("std_msgs/Header header" + "\n")
                    f.write("int32 seq" + "\n")
                fields = symbol.get_fields()
                fields.sort(key=field_byte_order)
                for field in symbol.get_fields():
                    typename = field.get_type_name()
                    fn = field.get_ros_name()
                    if fn not in v_names.keys():
                        v_names[fn] = 0
                    else:
                        v_names[fn] += 1
                        fn = fn + "_" + str(v_names[fn])
                    f.write(typename + " " + fn + "\n")
                f.close()

            except (IOError):
                self.get_logger().error("error writing msg file: " + fn)

    def get_location(self):
        '''
        Creates and returns the directory to write the messages file.

        Returns:
            msgs_src_dir (str): The name of the directory where the messages files will be written
        '''
        mpkg_name = "src/cfe_ros2_bridge_plugin/"
        pkg_name = "cfe_msgs/"
        p_up = "/../../../../"
        msgs_src_dir = self._cfs_msgs_dir + p_up + mpkg_name + pkg_name
        return msgs_src_dir


def main(args=None):
    rclpy.init(args=args)
    converter = CfeMsgConverter()
    converter.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
