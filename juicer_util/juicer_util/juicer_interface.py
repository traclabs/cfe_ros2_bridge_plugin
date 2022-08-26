import os

from rcl_interfaces.msg import ParameterDescriptor

from fsw_ros2_bridge.telem_info import TelemInfo
from fsw_ros2_bridge.command_info import CommandInfo

from juicer_util.juicer_database import JuicerDatabase


class JuicerInterface():

    def __init__(self, node):

        self._node = node
        self._node.get_logger().info("Loading message data from Juicer SQLite databases")

        self._node.declare_parameters(
          namespace="",
          parameters=[
            ('plugin_params.juicer_db', [], ParameterDescriptor(name='plugin_params.juicer_db',
                                                                dynamic_typing=True))
          ]
        )

        self._juicer_db = self._node.get_parameter('plugin_params.juicer_db'). \
            get_parameter_value().string_array_value

        self._telem_info = []
        self._command_info = []
        self._field_name_map = dict()
        self._symbol_name_map = dict()
        for db in self._juicer_db:

            if '~' in db:
                db = os.path.expanduser(db)

            self._node.get_logger().info("Parsing juicer db: " + db)

            self._db_data = JuicerDatabase(node, db)
            self._db_data.load_data()
            db_field_name_map = self._db_data.get_field_name_map()
            db_symbol_name_map = self._db_data.get_symbol_name_map()
            # field_name_map needs to be combined
            for key in db_field_name_map.keys():
                # TODO: check name and ros_name to see if they need to be updated
                # for now just add it to the combined list
                field = db_field_name_map[key]
                if key in self._field_name_map.keys():
                    self._node.get_logger().info("field name collision with " + key)
                else:
                    self._field_name_map[key] = field

            for key in db_symbol_name_map.keys():
                symbol = db_symbol_name_map[key]
                # TODO: do we want to change the name to make it unique?
                #       problem with that is some are default structs
                #       for example CFE_MSG_CommandHeader
                #       for now just keep the first one that we get
                if key in self._symbol_name_map.keys():
                    self._node.get_logger().info("symbol name collision with " + key)
                else:
                    self._symbol_name_map[key] = symbol

                if symbol.get_should_output():
                    if symbol.get_is_command():
                        c_key = symbol.get_name()
                        c_msg_type = symbol.get_ros_name()
                        c_topic = symbol.get_ros_topic()
                        c = CommandInfo(c_key, c_msg_type, c_topic, None)
                        self._command_info.append(c)
                        #self._node.get_logger().info("adding command: " + c_key)
                    elif symbol.get_is_telemetry():
                        t_key = symbol.get_name()
                        t_msg_type = symbol.get_ros_name()
                        t_topic = symbol.get_ros_topic()
                        t = TelemInfo(t_key, t_msg_type, t_topic)
                        self._telem_info.append(t)
                        #self._node.get_logger().info("adding telem: " + t_key)

    def get_telemetry_message_info(self):
        return self._telem_info

    def get_command_message_info(self):
        return self._command_info

    def get_msg_list(self):
        msg_list = []
        for key in self._symbol_name_map.keys():
            symbol = self._symbol_name_map[key]
            if len(symbol.get_fields()) > 0 and symbol.get_should_output():
                mn = symbol.get_ros_name()
                if mn[0].isupper():
                    msg_list.append(mn)
        return msg_list

    def get_topic_list(self):
        topic_list = []
        for key in self._symbol_name_map.keys():
            symbol = self._symbol_name_map[key]
            if len(symbol.get_fields()) > 0:
                tn = symbol.get_ros_topic()
                topic_list.append(tn)
        return topic_list

    def get_symbol_name_map(self):
        return self._symbol_name_map

    def get_field_name_map(self):
        return self._field_name_map


def field_sort_order(field):
    return field.get_bit_offset()
