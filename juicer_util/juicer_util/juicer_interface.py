import os
import importlib
import codecs

from rcl_interfaces.msg import ParameterDescriptor
from struct import unpack

from fsw_ros2_bridge.telem_info import TelemInfo
from fsw_ros2_bridge.command_info import CommandInfo

from juicer_util.juicer_database import JuicerDatabase


class JuicerInterface():

    def __init__(self, node, database_path):

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
        self._symbol_ros_name_map = dict()
        for db in self._juicer_db:

            if '~' in db:
                db = os.path.expanduser(db)
            else:
                db = database_path + db

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
                    ros_name = symbol.get_ros_name()
                    self._symbol_ros_name_map[ros_name] = symbol

                if symbol.get_should_output():
                    if symbol.get_is_command():
                        c_key = symbol.get_name()
                        c_msg_type = symbol.get_ros_name()
                        c_topic = symbol.get_ros_topic()
                        c = CommandInfo(c_key, c_msg_type, c_topic, None)
                        self._command_info.append(c)
                        # self._node.get_logger().info("adding command: " + c_key)
                    elif symbol.get_is_telemetry():
                        t_key = symbol.get_ros_name()
                        t_msg_type = symbol.get_ros_name()
                        t_topic = symbol.get_ros_topic()
                        t = TelemInfo(t_key, t_msg_type, t_topic)
                        self._telem_info.append(t)
                        # self._node.get_logger().info("adding telem: " + t_key)
                        # need to fix fields for CCSDSPrimaryHeader
            ccsds_prim_hdr = self._symbol_ros_name_map["CCSDSPrimaryHeader"]
            if ccsds_prim_hdr != None:
                two_bytes = self._symbol_ros_name_map["uint16"]
                for field in ccsds_prim_hdr.get_fields():
                    field.set_type_symbol(two_bytes)
                    field.set_little_endian(False)

        self._msg_list = self.set_up_msg_list()

    def get_telemetry_message_info(self):
        return self._telem_info

    def get_command_message_info(self):
        return self._command_info

    def get_msg_list(self):
        return self._msg_list

    def set_up_msg_list(self):
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

    def parse_packet(self, datagram, offset, ros_name, msg, msg_pkg):
        symbol = self._symbol_ros_name_map[ros_name]
        fields = symbol.get_fields()
        for field in fields:
            fsym = field.get_type_symbol()
            debug_name = field.get_ros_name() + "." + fsym.get_ros_name()
            self._node.get_logger().info("handle field " + debug_name)
            offs = offset + field.get_byte_offset()
            val = None
            # self._msg_list contains list of data types that need to be processed
            if fsym.get_ros_name() in self._msg_list:
                MsgType = getattr(importlib.import_module(msg_pkg + ".msg"),
                                  fsym.get_ros_name())
                fmsg = MsgType()
                val = self.parse_packet(datagram, offs, fsym.get_ros_name(), fmsg, msg_pkg)
                self._node.get_logger().info("Got value from recursive call for " + debug_name)
            else:
                if (fsym.get_ros_name() == 'string') or (fsym.get_ros_name() == 'char'):
                    # copy code from cfs_telem_receiver
                    ca = ""
                    for s in range(int(fsym.get_size())):
                        tf = unpack('c', datagram[(offs + s):(offs + s + 1)])
                        ca = ca + codecs.decode(tf[0], 'UTF-8')
                    val = ca
                    self._node.get_logger().info("Got value as a string - " + debug_name)
                else:
                    size = fsym.get_size()
                    fmt = self.get_unpack_format(fsym.get_ros_name(), field.get_endian())
                    tlm_field = unpack(fmt, datagram[offs:(offs + size)])
                    val = tlm_field[0]
                    self._node.get_logger().info("Unpacked value - " + debug_name + " using format " + fmt)
            # do something with val here
            if val is not None:
                setattr(msg, field.get_ros_name(), val)
                self._node.get_logger().info("Set " + field.get_ros_name() + " to value " + str(val))
            else:
                self._node.get_logger().info("Value for " + debug_name + " set through recursive call")
        return msg

    def parse_command(self, command_info, message, mid, code):
        self._node.get_logger().info("Handling command for " + command_info.get_msg_type())
        self._node.get_logger().info("Message: " + str(message))
        symbol = self._symbol_ros_name_map[command_info.get_msg_type()]
        packet = self.encode_command(symbol, message, mid, code)
        # for debugging - get the packet id
        # packetid = unpack(">H", packet[:2])
        # self._node.get_logger().info("packetid: " + str(packetid))
        return packet

    def encode_command(self, symbol, message, mid, code):
        packet = bytearray()
        fields = symbol.get_fields()
        fields.sort(key=field_sort_order)
        # start debug
        # mylist = " "
        # for field in fields:
        #     mylist += field.get_ros_name() + ":" + str(field.get_byte_offset()) + ", "
        # self._node.get_logger().info("---Doing fields " + mylist)
        # end debug
        for field in fields:
            fsym = field.get_type_symbol()
            fmsg = getattr(message, field.get_ros_name(), 0)
            debug_name = field.get_ros_name() + "." + fsym.get_ros_name()
            if len(fsym.get_fields()) == 0:
                self._node.get_logger().info("Storing concrete value for " + debug_name)
                fpacket = self.encode_data(field, fsym, fmsg)
                packet.extend(fpacket)
            else:
                self._node.get_logger().info("handle field " + debug_name)
                fpacket = self.encode_command(fsym, fmsg, mid, code)
                self._node.get_logger().info("Appending " + debug_name)
                packet.extend(fpacket)

        return packet

    def encode_data(self, field, fsym, fmsg):
        packet_size = fsym.get_size()
        packet = bytearray(packet_size)
        # process differently if string vs numeric?
        ros_name = fsym.get_ros_name()
        if ros_name.startswith("string") or ros_name.startswith("char"):
            # handle string
            string_b = fmsg.encode()
            packet[:fsym.get_size()] = string_b
            self._node.get_logger().info("Storing " + fmsg + " into " + field.get_ros_name())
        else:
            # handle numeric
            endian = 'big'
            if field.get_endian():
                endian = 'little'
            packet = fmsg.to_bytes(packet_size, endian)
            # TODO: need to handle floating point types differently
            self._node.get_logger().info("Storing " + str(fmsg) + " into " + field.get_ros_name() + " with endian " + endian)

        return packet

    def get_unpack_format(self, ros_name, little_endian):
        retval = "B"
        if ros_name == "uint64":
            retval = "Q"
        elif ros_name == "uint32":
            retval = "I"
        elif ros_name == "uint16":
            retval = "H"
        elif ros_name == "uint8":
            retval = "B"
        elif ros_name == "int64":
            retval = "q"
        elif ros_name == "int32":
            retval = "i"
        elif ros_name == "int16":
            retval = "h"
        elif ros_name == "int8":
            retval = "b"
        elif ros_name == "char":
            retval = "s"
        elif ros_name == "bool":
            retval = "?"
        else:
            self._logger.warn("Failed to get unpack format for " + ros_name)
        if little_endian:
            retval = "<" + retval
        else:
            retval = ">" + retval
        return retval


def field_sort_order(field):
    return field.get_byte_offset()
