import rclpy
from rclpy.node import Node
import os

from ament_index_python.packages import get_package_share_directory

from juicer_util.juicer_interface import JuicerInterface
from juicer_util.juicer_symbols import field_byte_order


class CfeMsgConverter(Node):
    def __init__(self):
        super().__init__('cfe_msg_converter')

        self.declare_parameter('cfs_root', 'cfe_msg_converter.cfs_root')
        self._cfs_root = self.get_parameter('cfs_root').get_parameter_value().string_value
        if '~' in self._cfs_root:
            self._cfs_root = os.path.expanduser(self._cfs_root)
        self.get_logger().info("cfs_root: " + self._cfs_root)

        self._cfs_msgs_dir = get_package_share_directory("cfe_msgs")

        self._msgs_src_dir = self.get_location()

        self.get_logger().info("cfe_msgs_dir: " + self._cfs_msgs_dir)

        pkg_name = "cfe_msg_converter"
        self._resource_path = get_package_share_directory(pkg_name) + "/resource"
        self.get_logger().info("resource_path: " + self._resource_path)

        self._cmake_template = self._resource_path + "/CMakeLists.template"
        self.get_logger().info("CMakeLists Template: " + self._cmake_template)

        resource_path = get_package_share_directory("cfe_msg_converter") + "/resource/"
        self._juicer_interface = JuicerInterface(self, resource_path)
        self._symbol_name_map = self._juicer_interface.get_symbol_name_map()
        self._msgs_list = self.create_messages(self._msgs_src_dir)

    def create_messages(self, msgs_dir):
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

    def write_cmake_lists_file(self):
        cmake_file = self.get_location() + "CMakeLists.txt"

        try:
            outf = open(cmake_file, 'w')
            msg_list = []

            with open(self._cmake_template) as inf:
                for ln in inf.readlines():
                    if ln == "# add dependencies here\n":
                        outf.write("rosidl_generate_interfaces(${PROJECT_NAME}\n")
                        outf.write("  \"msg/BinaryPktPayload.msg\"\n")

                        self.get_logger().info("Adding Juicer Msgs to CMakeLists.txt")
                        for m in self._msgs_list:
                            if m not in msg_list:
                                outf.write("  \"msg/" + m + ".msg\"\n")
                                msg_list.append(m)

                        outf.write("  DEPENDENCIES std_msgs\n")
                        outf.write(")\n")

                    else:
                        outf.write(ln)

            inf.close()
            outf.close()

        except (IOError):
            self.get_logger().error("error writing CMakeLists.txt")

    def get_location(self):
        mpkg_name = "src/cfe_ros2_bridge_plugin/"
        pkg_name = "cfe_msgs/"
        p_up = "/../../../../"
        msgs_src_dir = self._cfs_msgs_dir + p_up + mpkg_name + pkg_name
        return msgs_src_dir


def main(args=None):
    rclpy.init(args=args)
    converter = CfeMsgConverter()
    converter.write_cmake_lists_file()
    converter.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
