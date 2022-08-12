import rclpy
from rclpy.node import Node
import os

from juicer_util.juicer_fields import JuicerFieldEntry
from juicer_util.juicer_symbols import JuicerSymbolEntry

class CfeMsgConverter(Node):
    def __init__(self):
        super().__init__('cfe_msg_converter')

        self.declare_parameter('cfs_root', 'cfe_msg_converter.cfs_root')
        self.cfs_root = self.get_parameter('cfs_root').get_parameter_value().string_value
        if '~' in self.cfs_root:
            self.cfs_root = os.path.expanduser(self.cfs_root)
        self.get_logger().info("cfs_root: " + self.cfs_root)

    def create_messages(self, msgs_dir):
        for key in self.symbol_name_map.keys():
            if self.symbol_name_map[key].get_should_output():
                self.output_msg_file(self.symbol_name_map[key], msgs_dir)

        msg_list = []
        for key in self.symbol_name_map.keys():
            symbol = self.symbol_name_map[key]
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
            f = open(fn, "w")
            f.write("# cFE NAME: " + symbol.get_name() + "\n")
            f.write("# ROS topic: " + symbol.get_ros_topic() + "\n")
            if symbol.get_is_command():
                f.write("# Command message" + "\n")
            if symbol.get_is_telemetry():
                f.write("# Telemetry message" + "\n")
                f.write("int32 seq" + "\n")
            fields = symbol.get_fields()
            fields.sort(key=field_sort_order)
            for field in symbold.get_fields():
                typename = field.get_type_name()
                fn = field.get_ros_name()
                if fn not in v_names.keys():
                    v_names[fn] = 0
                else:
                    v_names[fn] += 1
                    fn = fn + "_" + str(v_names[fn])
                f.write(typename + " " + fn + "\n")
            f.close()

def main(args=None):
    rclpy.init(args=args)
    converter = CfeMsgConverter()
    #converter.writeCMakeListsFile()
    converter.desroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
