"""
.. module:: cfe_ros2_bridge_plugin.juicer_util.juicer_interface
   :synopsis: Serves as the main interface to access juicer database information

.. moduleauthor:: Tod Milam

"""
import os
import struct
import sys
import importlib
import codecs
import copy

from rcl_interfaces.msg import ParameterDescriptor
from struct import unpack, pack

from fsw_ros2_bridge.telem_info import TelemInfo
from fsw_ros2_bridge.command_info import CommandInfo

from juicer_util.juicer_database import JuicerDatabase


class JuicerInterface():
    """This class is the main way of accessing information from the juicer database.

    Attributes
    ----------
    node : rosnode
        the ROS2 node
    juicer_db : dict
        list of juicer database files to read
    telem_info : dict
        list of TelemInfo objects
    command_info : dict
        list of CommandInfo objects
    field_name_map : dict
        list of field entries from the database
    symbol_name_map : dict
        list of symbol entries from the database mapped to name
    symbol_ros_name_map : dict
        list of symbol entries from the database mapped to ROS2 name
    msg_list : list
        list of ROS2 messages to write to file

    Methods
    -------
    get_telemetry_message_info():
        Returns the list of telemetry objects
    get_command_message_info():
        Returns the list of command objects
    reconcile_telem_info(tlm_info, tlm_dict):
        Combines data from tlm_info and tlm_dict into singe structure
    reconcile_command_info(cmd_info, cmd_dict):
        Combines data from cmd_info and cmd_dict into singe structure
    get_msg_list():
        Returns the list of ROS2 messages to be created
    set_up_msg_list():
        Create the list of ROS2 messages to write to file
    get_topic_list():
        Return the list of ROS2 topics
    get_symbol_name_map():
        Returns the list of symbols mapped by name
    get_symbol_ros_name_map():
        Returns the list of symbols mapped by ROS2 name
    get_field_name_map():
        Returns the list of fields mapped by name
    parse_packet(datagram, offset, ros_name, msg, msg_pkg):
        Parse data packet and return ROS2 structure
    parse_command(command_info, message, mid, code):
        Parse the ROS2 command into cFS packet
    encode_command(symbol, message, mid, code):
        Convert ROS2 command message into cFS command packet
    encode_data(field, fsym, fmsg):
        Encode data into byte array
    get_unpack_format(ros_name, little_endian):
        Returns the format for decoding data
    get_symbol_info(name):
        Return the symbol object for the given symbol name
    """

    def __init__(self, node, database_path):

        self._node = node
        self._node.get_logger().debug("Loading message data from Juicer")

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

            if db.startswith('~'):
                db = os.path.expanduser(db)
            elif not db.startswith('/'):
                # need to use database_path or relative path
                db2 = database_path + db
                if not os.path.isfile(db2):
                    db = os.path.join(os.getcwd(), db)
                else:
                    db = db2

            self._node.get_logger().debug("Parsing juicer db: " + db)

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
                    self._node.get_logger().debug("field name collision with " + key)
                else:
                    self._field_name_map[key] = field

            for key in db_symbol_name_map.keys():
                symbol = db_symbol_name_map[key]
                # TODO: do we want to change the name to make it unique?
                #       problem with that is some are default structs
                #       for example CFE_MSG_CommandHeader
                #       for now just keep the first one that we get
                if key in self._symbol_name_map.keys():
                    self._node.get_logger().debug("symbol name collision with " + key)
                else:
                    self._symbol_name_map[key] = symbol
                    ros_name = symbol.get_ros_name()
                    self._symbol_ros_name_map[ros_name] = symbol

                if symbol.get_should_output():
                    if symbol.get_is_command():
                        c_key = symbol.get_name()
                        c_msg_type = symbol.get_ros_name()
                        c_topic = symbol.get_ros_topic()
                        c = CommandInfo(c_key, c_msg_type, c_topic, None, 0)
                        self._command_info.append(c)
                    elif symbol.get_is_telemetry():
                        t_key = symbol.get_ros_name()
                        t_msg_type = symbol.get_ros_name()
                        t_topic = symbol.get_ros_topic()
                        t = TelemInfo(t_key, t_msg_type, t_topic, 0)
                        self._telem_info.append(t)
            ccsds_prim_hdr = self._symbol_ros_name_map["CCSDSPrimaryHeader"]
            # need to fix fields for CCSDSPrimaryHeader
            if ccsds_prim_hdr is not None:
                two_bytes = self._symbol_ros_name_map["uint16"]
                for field in ccsds_prim_hdr.get_fields():
                    field.set_type_symbol(two_bytes)
                    field.set_little_endian(False)
            # need to fix time field for CFEMSGTelemetrySecondaryHeadert
            telem_sec_hdr = self._symbol_ros_name_map["CFEMSGTelemetrySecondaryHeadert"]
            if telem_sec_hdr is not None:
                for field in telem_sec_hdr.get_fields():
                    field.set_little_endian(False)

        self._msg_list = self.set_up_msg_list()
        self._seq = 0 # Seq counter

    def get_telemetry_message_info(self):
        '''
        Returns the list of telemetry objects

        Returns:
            telem_info (dict): The list of telemetry objects
        '''
        return self._telem_info

    def get_command_message_info(self):
        '''
        Returns the list of command objects

        Returns:
            command_info (dict): The list of command objects
        '''
        return self._command_info

    def reconcile_telem_info(self, tlm_info, tlm_dict):
        '''
        Combines data from tlm_info and tlm_dict into singe structure

        Args:
            tlm_info (dict): The list of telemetry objects from juicer database
            tlm_dict (dict): A list of telemetry objects with additional data

        Returns:
            tlm_info (dict): A list of combined telemetry information
        '''
        # need to create new TelemInfo entries for each entry in tlm_dict
        telem_info = []
        for key in tlm_dict.keys():
            td = tlm_dict[key]
            struct_name = td["structure"]
            t_key = key
            t_msg_type = struct_name
            t_topic = td["topic_name"]
            t_port = 0
            if "port" in td:
                t_port = td["port"]
            t = TelemInfo(t_key, t_msg_type, t_topic, t_port)
            telem_info.append(t)
        return telem_info

    def reconcile_command_info(self, cmd_info, cmd_dict):
        '''
        Combines data from cmd_info and cmd_dict into singe structure

        Args:
            cmd_info (dict): The list of command objects from juicer database
            cmd_dict (dict): A list of command objects with additional data

        Returns:
            cmd_info (dict): A list of combined command information
        '''
        # need to create new CommandInfo entries for each entry in cmd_dict
        command_info = []
        for key in cmd_dict.keys():
            cd = cmd_dict[key]
            struct_name = cd["structure"]
            c_key = key
            c_msg_type = struct_name
            # symbol = self._symbol_ros_name_map[struct_name]
            c_topic = cd["topic_name"]
            c_port = 0
            if "port" in cd:
                c_port = cd["port"]
            c = CommandInfo(c_key, c_msg_type, c_topic, None, c_port)
            command_info.append(c)
        return command_info

    def get_msg_list(self):
        '''
        Returns the list of ROS2 messages to be created

        Returns:
            msg_list (): The list of ROS2 messages to write to file
        '''
        return self._msg_list

    def set_up_msg_list(self):
        '''
        Create the list of ROS2 messages to write to file

        Returns:
            msg_list (list): The list of ROS2 messages to write to file
        '''
        msg_list = []
        for key in self._symbol_name_map.keys():
            symbol = self._symbol_name_map[key]
            if len(symbol.get_fields()) > 0 and symbol.get_should_output():
                mn = symbol.get_ros_name()
                if mn[0].isupper():
                    msg_list.append(mn)
        return msg_list

    def get_topic_list(self):
        '''
        Return the list of ROS2 topics

        Returns:
            The list of ROS2 topics
        '''
        topic_list = []
        for key in self._symbol_name_map.keys():
            symbol = self._symbol_name_map[key]
            if len(symbol.get_fields()) > 0:
                tn = symbol.get_ros_topic()
                topic_list.append(tn)
        return topic_list

    def get_symbol_name_map(self):
        '''
        Returns the list of symbols mapped by name

        Returns:
            A mapping of symbol to symbol name
        '''
        return self._symbol_name_map

    def get_symbol_ros_name_map(self):
        '''
        Returns the list of symbols mapped by ROS2 name

        Returns:
            A mapping of symbol to the symbol's ROS2 name
        '''
        return self._symbol_ros_name_map

    def get_field_name_map(self):
        '''
        Returns the list of fields mapped by name

        Returns:
            A mapping of field to field name
        '''
        return self._field_name_map

    def parse_packet(self, datagram, offset, ros_name, msg, msg_pkg):
        '''
        Parse data packet and return ROS2 structure

        Args:
            datagram (bytearray): The incoming data
            offset (int): Offset into datagram to start processing
            ros_name (str): The ROS2 name of the message to process
            msg: The ROS2 message to populate
            msg_pkg (str): The package name of the ROS2 message to populate

        Returns:
            msg: The populated ROS2 message
        '''
        symbol = self._symbol_ros_name_map[ros_name]
        fields = symbol.get_fields()
        for field in fields:
            fsym = field.get_type_symbol()
            is_array, length = field.get_is_array()
            debug_name = field.get_ros_name() + "." + fsym.get_ros_name()
            offs = offset + field.get_byte_offset()
            self._node.get_logger().debug("handle field " + debug_name + " of length "
                                          + str(length) + " for field " + field.get_ros_name()
                                          + " with offset " + str(offs))
            val = None
            fmt = "not set"
            try:
                # self._msg_list contains list of data types that need to be processed
                if fsym.get_ros_name() in self._msg_list:
                    MsgType = getattr(importlib.import_module(msg_pkg + ".msg"),
                                      fsym.get_ros_name())
                    # this may be an array, so prepare for it
                    aryval = []
                    size = fsym.get_size()
                    fmsg = MsgType()
                    self._node.get_logger().debug("Got val from recursive call for "
                                                  + debug_name + ", " + fsym.get_ros_name())
                    for x in range(length):
                        val = self.parse_packet(datagram, offs + x * size, fsym.get_ros_name(),
                                                fmsg, msg_pkg)
                        aryval.append(copy.deepcopy(val))
                    if length > 1:
                        val = copy.deepcopy(aryval)
                else:
                    if (fsym.get_ros_name() == 'string') or (fsym.get_ros_name() == 'char'):
                        # copy code from cfs_telem_receiver
                        ca = ""
                        size = int(length)
                        self._node.get_logger().debug("unpacking " + str(size) + " char string")
                        num_decoded = 0
                        for s in range(size):
                            start = offs + s
                            end = start + 1
                            if end > len(datagram):
                                self._node.get_logger().error("ERROR: trying to read past EOB for "
                                                              + debug_name + "!")
                                break
                            tf = unpack('c', datagram[start:end])
                            ca = ca + codecs.decode(tf[0], 'UTF-8')
                            num_decoded = num_decoded + 1
                        val = ca
                        self._node.get_logger().debug("Got value as a string - " + debug_name
                                                      + " with " + str(num_decoded) + " items")
                        if num_decoded == 0:
                            val = None
                    else:
                        # this may be an array, so prepare for it
                        aryval = []
                        size = fsym.get_size()
                        fmt = self.get_unpack_format(fsym.get_ros_name(), field.get_endian())
                        self._node.get_logger().debug("unpack format is " + fmt
                                                      + ", length is " + str(length))
                        num_decoded = 0
                        for x in range(int(length)):
                            start = offs + size * x
                            end = offs + size * (x + 1)
                            if end > len(datagram):
                                self._node.get_logger().debug("ERROR: trying to read past EOB for "
                                                              + debug_name + "!")
                                break
                            self._node.get_logger().debug("unpack range is from " + str(start)
                                                          + " to " + str(end))
                            tlm_field = unpack(fmt, datagram[start:end])
                            val = tlm_field[0]
                            aryval.append(copy.deepcopy(val))
                            num_decoded = num_decoded + 1
                        if length > 1:
                            val = copy.deepcopy(aryval)
                        self._node.get_logger().debug("Unpacked value - " + debug_name
                                                      + " using format " + fmt
                                                      + " with " + str(num_decoded) + " items")
                        if num_decoded == 0:
                            val = None
                # do something with val here
                if val is not None:
                    setattr(msg, field.get_ros_name(), val)
                    self._node.get_logger().debug("Set " + field.get_ros_name()
                                                  + " to value " + str(val)
                                                  + ", debug_name: " + debug_name)

                else:
                    self._node.get_logger().debug("Value for " + debug_name
                                                  + " set through recursive call")
            except (TypeError):
                self._node.get_logger().error("Error unpacking - " + debug_name
                                              + " with format " + fmt)
        return msg

    def parse_command(self, command_info, message, mid, code):
        '''
        Parse the ROS2 command into cFS packet

        Args:
            command_info (CommandInfo): The command info for this command
            message: The incoming ROS2 message
            mid (int): The cFS message id for this command
            code (int): The cFS command code

        Returns:
            packet (bytearray): Populated cFS command packet
        '''
        if not command_info.get_msg_type():
            self._node.get_logger().info("Handling command for " + command_info.get_key()
                                         + " with generic Binary handler")
            return self.encode_binary_command(message, mid, code)
        self._node.get_logger().debug("Handling command for " + command_info.get_key() +
                                      " of type " + command_info.get_msg_type())
        self._node.get_logger().debug("Message: " + str(message))
        symbol = self._symbol_ros_name_map[command_info.get_msg_type()]
        packet = self.encode_command(symbol, message, mid, code)
        return packet

    def encode_binary_command(self, message, mid, code):
        self._seq = self._seq+1 # TODO: Is seq set on nominal commands being sent? Should this var be used to override if not?

        self._node.get_logger().info("mid is " + str(mid) +"="+ str(int(mid,0)) + f", seq={self._seq}, len={len(message.data)+8-7} from {len(message.data)}, code={code}")

        mid=int(mid,0)
        if (mid & 0x1000):
            self._node.get_logger().info("Encoding as command")
        
            hdr = struct.pack(">HHHh",
                              mid | 0x800, # Set secondary hdr flag
                              self._seq, # VERIFY
                              len(message.data)+8-7,
                              code # function code,
                              #                          0 # spare
                              )
        else:
            self._node.get_logger().info("Encoding as tlm")
        
            hdr = struct.pack(">HHHIHI",
                              mid | 0x800,
                              self._seq, # VERIFY
                              len(message.data)+16-7,
                              # NOTE: CCSDS Time Format may vary between Cfe Configs. This logic should come from juicer (TODO)
                              0, # TODO: Seconds, 32-bit
                              0, # TODO: Subseconds, 16-bit
                              0, # 32-bit Spare
                              )


        # Because python is so clear at binary data manipulation
        rtv = hdr + b''.join(message.data)

        self._node.get_logger().info("hdr    : " + hdr.hex() ) #str(hdr));
        self._node.get_logger().info("sendbin: " + rtv.hex() ) #str(rtv)) )
        #self._node.get_logger().info("data   : " + str(message.data) )
        
        return rtv

        # QUESTION: If this fn builds packet /w header ... then why is mid/code not used in original encode_command?
    
    def encode_command(self, symbol, message, mid, code):
        '''
        Convert ROS2 command message into cFS command packet

        Args:
            symbol (JuicerSymbolEntry): Symbol associated with this command
            message: ROS2 command message
            mid (int): cFS command message id
            code (int): cFS command code

        Returns:
            packet (bytearray): The cFS command packet
        '''
        packet = bytearray()
        fields = symbol.get_fields()
        fields.sort(key=field_sort_order)
        for field in fields:
            fsym = field.get_type_symbol()
            fmsg = getattr(message, field.get_ros_name(), 0)
            debug_name = field.get_ros_name() + "." + fsym.get_ros_name()
            if len(fsym.get_fields()) == 0:
                self._node.get_logger().debug("Storing concrete value for " + debug_name)
                fpacket = self.encode_data(field, fsym, fmsg)
                packet.extend(fpacket)
            else:
                fpacket = self.encode_command(fsym, fmsg, mid, code)
                self._node.get_logger().debug("fpacket " + str(fpacket))
                packet.extend(fpacket)

        return packet

    def encode_data(self, field, fsym, fmsg):
        '''
        Encode data into byte array

        Args:
            field (JuicerFieldEntry): The field to encode
            fsym (JuicerSymbolEntry): The data type to encode
            fmsg: The data to encode

        Returns:
            packet (bytearray): The encoded data
        '''
        packet_size = fsym.get_size()
        packet = bytearray(packet_size)
        # process differently if string vs numeric?
        ros_name = fsym.get_ros_name()
        try:
            if ros_name.startswith("string") or ros_name.startswith("char"):
                # handle string
                string_b = fmsg.encode()
                packet[:fsym.get_size()] = string_b
                self._node.get_logger().debug("Storing " + fmsg + " into " + field.get_ros_name())
            elif ros_name.startswith("float"):
                fmt = self.get_unpack_format(fsym.get_ros_name(), 1 - field.get_endian())
                packet = pack(fmt, fmsg)
            else:
                # handle numeric
                endian = 'big'
                if field.get_endian():
                    endian = 'little'
                packet = fmsg.to_bytes(packet_size, endian)
                # TODO: need to handle floating point types differently
                self._node.get_logger().debug("Storing " + str(fmsg) + " into "
                                              + field.get_ros_name()
                                              + " with endian " + endian)

        except (TypeError):
            self._node.get_logger().error("problem tryin to encode data: " + ros_name)

        return packet

    def get_unpack_format(self, ros_name, little_endian):
        '''
        Returns the format for decoding data

        Args:
            ros_name (str): The ROS2 name of the message
            little_endian (bool): True if data is little endian

        Returns:
            retval (str): The unpack format code
        '''
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
        elif ros_name == "float32":
            retval = "f"
        elif ros_name == "float64":
            retval = "d"
        else:
            self._node.get_logger().warn("Failed to get unpack format for " + ros_name)
        if little_endian:
            retval = "<" + retval
        else:
            retval = ">" + retval

        if ros_name == "float32":
            retval = "<f"

        return retval

    def get_symbol_info(self, name):
        '''
        Return the symbol object for the given symbol name

        Args:
            name (str): The name of the symbol

        Returns:
            symbol (JuicerSymbolEntry): The symbol object
        '''
        symbol = self._symbol_name_map[name]
        return symbol


def field_sort_order(field):
    '''
    Helper function to sort fields by byte offset

    Args:
        field (JuicerFieldEntry): The field to sort

    Returns:
        The byte offset of the field
    '''
    return field.get_byte_offset()
