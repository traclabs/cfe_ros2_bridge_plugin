import rclpy
from brash_bridge.juicer_interface import JuicerInterface, field_sort_order


class FSWPlugin(JuicerInterface):

    def __init__(self):
        self._logger = rclpy.logging.get_logger("JuicerConverter")
        JuicerInterface.__init__(self)

    def create_ros_msgs(self, msgs_dir):
        for key in self._symbol_name_map.keys():
            # self.output_ros_msg_file(self._symbol_name_map[key], msgs_dir)
            if self._symbol_name_map[key].get_should_output():
                self.output_ros_msg_file(self._symbol_name_map[key], msgs_dir)

        msg_list = []
        cmd_list = []
        tlm_list = []
        for key in self._symbol_name_map.keys():
            symbol = self._symbol_name_map[key]
            if len(symbol.get_fields()) > 0 and symbol.get_should_output():
                mn = symbol.get_ros_name()
                if mn[0].isupper():
                    msg_list.append(mn)
                    if symbol.get_is_command():
                        item = {"ros_name": mn, "cfe_mid": "0x1200"}
                        cmd_list.append(item)
                    if symbol.get_is_telemetry():
                        item = {"ros_name": mn, "cfe_mid": "0x200"}
                        tlm_list.append(item)
        # cfg_items = {"commands": cmd_list, "telemetry": tlm_list}
        # print(yaml.dump(cfg_items))

        return msg_list

    def output_ros_msg_file(self, symbol, msgs_dir):
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
            # f.write("int32 seq\n")
            fields = symbol.get_fields()
            fields.sort(key=field_sort_order)
            for field in symbol.get_fields():
                typename = field.get_type_name()
                fn = field.get_ros_name()
                if fn not in v_names.keys():
                    v_names[fn] = 0
                else:
                    v_names[fn] += 1
                    # print(mn + " has duplicate field name " + fn)
                    fn = fn + "_" + str(v_names[fn])
                f.write(typename + " " + fn + "\n")
            f.close()
