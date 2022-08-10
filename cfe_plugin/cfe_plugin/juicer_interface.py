import json
import rclpy
import sqlite3
from sqlite3 import Error

from rcl_interfaces.msg import ParameterDescriptor
from rclpy.parameter import Parameter

from fsw_ros2_bridge.fsw_plugin_interface import *
from fsw_ros2_bridge.telem_info import TelemInfo
from fsw_ros2_bridge.command_info import CommandInfo
from cfe_plugin.juicer_fields import JuicerFieldEntry
from cfe_plugin.juicer_symbols import JuicerSymbolEntry


class JuicerInterface():

    def __init__(self, node):

        self.node = node
        self.node.get_logger().info("Loading message data from Juicer SQLite databases")

        self.node.declare_parameters(
          namespace="",
          parameters=[
            ('plugin_params.juicer_db', [], ParameterDescriptor(name='plugin_params.juicer_db', dynamic_typing=True))
          ]
        )

        self.juicer_db = self.node.get_parameter('plugin_params.juicer_db').get_parameter_value().string_array_value

        for db in self.juicer_db:
            self.node.get_logger().info("Parsing juicer db: " + db)

            self.conn = self.create_connection(db)
            self.field_name_map = dict()
            self.symbol_name_map = dict()
            self.symbol_id_map = dict()

            self.telem_info = []
            self.command_info = []
            self.recv_map = {}

            # self.loadConfig()
            self.loadData()

            for key in self.symbol_name_map.keys():
                symbol = self.symbol_name_map[key]
                if symbol.get_should_output():
                    if symbol.get_is_command():
                        #print("Found command " + symbol.get_name())
                        c_key = symbol.get_name()
                        c_msg_type = symbol.get_ros_name()
                        c_topic = symbol.get_ros_topic()
                        #c = CommandInfo(c_key, c_msg_type, c_topic)
                        #self.command_info.append(c)
                    elif symbol.get_is_telemetry():
                        t_key = symbol.get_name()
                        t_msg_type = symbol.get_ros_name()
                        t_topic = symbol.get_ros_topic()
                        t = TelemInfo(t_key, t_msg_type, t_topic)
                        self.telem_info.append(t)

    def create_connection(self, db_file):
        """ create a database connection to the SQLite database
            specified by the db_file
        :param db_file: database file
        :return: Connection object or None
        """
        conn = None
        try:
            conn = sqlite3.connect(db_file)
        except Error as e:
            self.node.get_logger().error(e)

        return conn

    def retrieve_all_fields(self):
        """
        Query all rows in the fields table
        :return: A mapping of field name to field object
        """
        cur = self.conn.cursor()
        cur.execute("SELECT * FROM fields")

        rows = cur.fetchall()
        for row in rows:
            my_field = JuicerFieldEntry(row[0], row[1], row[2], row[3], row[4], row[5], row[6], row[7])
            self.field_name_map[my_field.get_name()] = my_field
            symbol = self.symbol_id_map[my_field.getSymbol()]
            if symbol is not None:
                typeid = my_field.get_type()
                my_field.setTypeSymbol(self.symbol_id_map[typeid])
                symbol.addField(my_field)
        return self.field_name_map

    def retrieve_all_symbols(self):
        """
        Query all rows in the symbols table
        :return: A mapping of symbol name to symbol object
        """
        cur = self.conn.cursor()
        cur.execute("SELECT * FROM symbols")
        rows = cur.fetchall()

        for row in rows:
            my_symbol = JuicerSymbolEntry(row[0], row[1], row[2], row[3])
            self.symbol_id_map[my_symbol.getId()] = my_symbol
            if not my_symbol.get_name().startswith("_"):
                self.symbol_name_map[my_symbol.get_name()] = my_symbol
        return self.symbol_id_map

    def loadData(self):
        self.retrieve_all_symbols()
        self.retrieve_all_fields()
        self.prune_symbols_and_fields()
        self.mark_cmd_tlm_symbols()
    # def loadConfig(self):
    #     with open(self.jsonConfigFile, "r") as jsonfile:
    #         jsonConfig = json.load(jsonfile)
    #         self.cmdIds = jsonConfig["commands"]
    #         self.tlmIds = jsonConfig["telemetry"]

    def prune_symbols_and_fields(self):
        self.node.get_logger().info("Pruning out things that aren't needed.")
        self.empty_symbols = []
        for key in self.symbol_name_map.keys():
            symbol = self.symbol_name_map[key]
            if len(symbol.get_fields()) == 0:
                self.empty_symbols.append(symbol)
                # for some reason some messages need to be added twice, so just automatically do it for all of them
                self.empty_symbols.append(symbol)
        self.node.get_logger().info("There are " + str(len(self.empty_symbols)) + " empty symbols")
        for symbol in self.empty_symbols:
            mn = symbol.get_ros_name()
            # if it starts with lower case then it is ROS2 native type so ignore it
            if mn[0].isupper():
                altSym = self.find_alternative_symbol(symbol)
                if altSym is not None:
                    self.empty_symbols.remove(symbol)
                    symbol.set_alternative(altSym)
                #else:
                #print("Unable to find an alternative for " + symbol.get_name())
        self.node.get_logger().info("There are " + str(len(self.empty_symbols)) + " empty symbols left after pruning")

    def find_alternative_symbol(self, empty_symbol):
        #print("empty symbol " + empty_symbol.get_ros_name() + " from " + empty_symbol.get_name())
        for key in self.symbol_name_map.keys():
            symbol = self.symbol_name_map[key]
            if empty_symbol.get_name().startswith(symbol.get_name()):
                if not empty_symbol == symbol:
                    if empty_symbol.getSize() == symbol.getSize():
                        #print("Should replace " + empty_symbol.get_name() + " with " + symbol.get_name())
                        return symbol
                    else:
                        self.node.get_logger().warn("Can't replace " + empty_symbol.get_name() + " with " + symbol.get_name())
                        self.node.get_logger().warn("wrong size " + str(empty_symbol.getSize()) + " vs " + str(symbol.getSize()))
            elif empty_symbol.get_name() == "CFE_EVS_SetEventFormatMode_Payload_t" and symbol.get_name() == "CFE_EVS_SetEventFormatCode_Payload":
                self.node.get_logger().info("Handling the SetEventFormatMode vs SetEventFormatCode problem")
                return symbol
        return None

    def get_telemetry_message_info(self):
        return self.telem_info

    def get_command_message_info(self):
        return self.command_info
# NOTE: getLatestData is in juicer_bridge.py
#    def getLatestData(self, key):
#return self.recv_map[key].getLatestData()
#        return None

    def get_msg_list(self):
        msg_list = []
        for key in self.symbol_name_map.keys():
            symbol = self.symbol_name_map[key]
            if len(symbol.get_fields()) > 0 and symbol.get_should_output():
                mn = symbol.get_ros_name()
                if mn[0].isupper():
                    msg_list.append(mn)
        return msg_list

    def get_topic_list(self):
        topic_list = []
        for key in self.symbol_name_map.keys():
            symbol = self.symbol_name_map[key]
            if len(symbol.get_fields()) > 0:
                tn = symbol.get_ros_topic()
                topic_list.append(tn)
        return topic_list

    def mark_cmd_tlm_symbols(self):
        self.node.get_logger().info("Marking symbols that are necessary for messages.")
        self.empty_symbols = []
        for key in self.symbol_name_map.keys():
            symbol = self.symbol_name_map[key]
            if symbol.get_is_command() or symbol.get_is_telemetry():
                self.mark_output_symbol(symbol)

    def mark_output_symbol(self, symbol):
        symbol.set_should_output(True)
        fields = symbol.get_fields()
        for field in fields:
            field_symbol = field.get_type_symbol()
            self.mark_output_symbol(field_symbol)


def field_sort_order(field):
    return field.getBitOffset()
