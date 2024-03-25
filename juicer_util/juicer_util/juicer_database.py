"""
.. module:: cfe_ros2_bridge_plugin.juicer_util.juicer_database
   :synopsis: Serves as the interface to the sqlite datase file

.. moduleauthor:: Tod Milam

"""


import sqlite3
import sys
from sqlite3 import Error
from juicer_util.juicer_fields import JuicerFieldEntry
from juicer_util.juicer_symbols import JuicerSymbolEntry
from juicer_util.juicer_symbols import field_byte_order


class JuicerDatabase():
    """This class serves as the interface to the sqlite database.

    Attributes
    ----------
    node : rosnode
        the ROS2 node
    db_file : str
        the name of the sqlite file
    field_name_map : dict
        list of field entries from the database
    symbol_name_map : dict
        list of symbol entries from the database mapped to name
    symbol_id_map : dict
        list of symbol entries from the database mapped to id

    Methods
    -------
    create_connection(db_file):
        Returns a connection to the specified database.
    retrieve_all_symbols():
        Query all rows in the symbols table
    retrieve_all_fields():
        Query all rows in the fields table
    load_data():
        Load the data from the database
    prune_symbols_and_fields():
        Remove symbols and fields that are not used by either telemetry or commands.
    find_alternative_symbol(empty_symbol):
        Find a symbol to replace the empty symbol that may be a typedef.
    mark_cmd_tlm_symbols():
        Mark all symbols that are either telemetry or command data types.
    mark_output_symbol(symbol):
        Mark this symbol as one that should be written to a msg file and mark each of its fields.
    get_symbol_name_map():
        Return the symbol list as a map of symbol name to symbol object.
    get_field_name_map():
        Return the field list as a map of field name to field object.
    handle_duplicate_fields():
        Check all fields for duplicate names and rename as necessary.
    rename_duplicate_fields(symbol):
        Rename all fields that have the given name.
    """

    def __init__(self, node, db_file, use_native_endian = False):
        '''
        Initializes the attributes for the object.

        Args:
            node (rosnode): The ROS2 node
            db_file (str): The name of the sqlite file
            use_native_endian (bool): Use native endian rather than juicer value
        '''
        self._node = node
        self._node.get_logger().debug("Loading message data from Juicer SQLite databases")
        self._db_file = db_file
        self._field_name_map = dict()
        self._symbol_name_map = dict()
        self._symbol_id_map = dict()
        self._use_native_endian = use_native_endian
        if use_native_endian:
            if sys.byteorder == 'little':
                self._native_little_endian = True
            else:
                self._native_little_endian = False

    def create_connection(self, db_file):
        '''
        Create a database connection to the SQLite database
            specified by the db_file

        Args:
            db_file (str): database file name

        Returns:
            conn (sqlite connection): Connection object or None
        '''
        conn = None
        try:
            conn = sqlite3.connect(db_file)
        except Error as e:
            self._node.get_logger().error(str(e))

        return conn

    def retrieve_all_symbols(self):
        '''
        Query all rows in the symbols table

        Returns:
            symbol_id_map (dict): A mapping of symbol name to symbol object
        '''
        cur = self._conn.cursor()
        cur.execute("SELECT * FROM symbols")
        rows = cur.fetchall()

        for row in rows:
            my_symbol = JuicerSymbolEntry(self._node, row[0], row[1], row[2], row[3])
            self._symbol_id_map[my_symbol.get_id()] = my_symbol
            if not my_symbol.get_name().startswith("_"):
                self._symbol_name_map[my_symbol.get_name()] = my_symbol
        cur.close()
        return self._symbol_id_map

    def retrieve_all_fields(self):
        '''
        Query all rows in the fields table

        Returns:
            field_name_map (dict): A mapping of field name to field object
        '''
        cur = self._conn.cursor()
        cur.execute("SELECT * FROM fields")

        rows = cur.fetchall()
        last_id = 0
        last_endian = 0
        for row in rows:
            my_field = JuicerFieldEntry(self._node, row[0], row[1], row[2], row[3],
                                        row[4], row[5], row[6], row[7])
            if self._use_native_endian:
                my_field.set_little_endian(self._native_little_endian)
                self._node.get_logger().debug("Using native endian for field " + my_field.get_name() + ".")
            last_id = my_field.get_id()
            last_endian = my_field.get_endian()
            self._field_name_map[my_field.get_name()] = my_field
            symbol = self._symbol_id_map[my_field.get_symbol()]
            if symbol is not None:
                typeid = my_field.get_type()
                my_field.set_type_symbol(self._symbol_id_map[typeid])
                symbol.add_field(my_field)
            else:
                self._node.get_logger().debug("Can't find symbol for field " + my_field.get_name())

        cur.close()
        # Need to add a field of type CCSDS_SpacePacket_t and add it to CFE_MSG_Message symbol
        if "CFE_MSG_Message" in self._symbol_name_map:
            symbol = self._symbol_name_map["CFE_MSG_Message"]
            fid = last_id + 1
            container_symbol = symbol.get_id()
            fname = "CCSDS"
            foffset = 0
            fsymbol = self._symbol_name_map["CCSDS_SpacePacket_t"]
            ftype = fsymbol.get_id()
            fendian = last_endian
            fbit_size = 0
            fbit_offset = 0
            my_field = JuicerFieldEntry(self._node, fid, container_symbol, fname, foffset, ftype,
                                        fendian, fbit_size, fbit_offset)
            self._field_name_map[my_field.get_name()] = my_field
            typeid = my_field.get_type()
            my_field.set_type_symbol(self._symbol_id_map[typeid])
            symbol.add_field(my_field)
        return self._field_name_map

    def load_data(self):
        '''
        Load the data from the database
        '''
        self._conn = self.create_connection(self._db_file)
        self.retrieve_all_symbols()
        self.retrieve_all_fields()
        self._conn.close()
        self.prune_symbols_and_fields()
        self.mark_cmd_tlm_symbols()
        self.handle_duplicate_fields()

    def prune_symbols_and_fields(self):
        '''
        Remove symbols and fields that are not used by either telemetry or commands.
        '''
        self._node.get_logger().debug("Pruning out things that aren't needed.")
        self._empty_symbols = []
        for key in self._symbol_name_map.keys():
            symbol = self._symbol_name_map[key]
            if len(symbol.get_fields()) == 0:
                self._empty_symbols.append(symbol)
                # for some reason some messages need to be added twice,
                # so just automatically do it for all of them
                self._empty_symbols.append(symbol)
        self._node.get_logger().debug("There are " + str(len(self._empty_symbols))
                                      + " empty symbols")
        for symbol in self._empty_symbols:
            mn = symbol.get_ros_name()
            # if it starts with lower case then it is ROS2 native type so ignore it
            if mn[0].isupper():
                altSym = self.find_alternative_symbol(symbol)
                if altSym is not None:
                    self._empty_symbols.remove(symbol)
                    symbol.set_alternative(altSym)
        self._node.get_logger().debug("There are " + str(len(self._empty_symbols))
                                      + " empty symbols left after pruning")

    def find_alternative_symbol(self, empty_symbol):
        '''
        Find a symbol to replace the empty symbol that may be a typedef.

        Args:
            empty_symbol (symbol): The empty symbol to be replaced

        Returns:
            symbol (symbol): The replacement symbol or None
        '''
        for key in self._symbol_name_map.keys():
            symbol = self._symbol_name_map[key]
            if empty_symbol.get_name().startswith(symbol.get_name()):
                if not empty_symbol == symbol:
                    if empty_symbol.get_size() == symbol.get_size():
                        return symbol
                    else:
                        self._node.get_logger().warn("Can't replace " + empty_symbol.get_name() +
                                                     " with " + symbol.get_name())
                        self._node.get_logger().warn("wrong size " + str(empty_symbol.get_size()) +
                                                     " vs " + str(symbol.get_size()))
            # handle special case where message name was misspelled (Mode vs Code)
            elif empty_symbol.get_name() == "CFE_EVS_SetEventFormatMode_Payload_t" and \
                    symbol.get_name() == "CFE_EVS_SetEventFormatCode_Payload":
                self._node.get_logger().debug("Handling SetEventFormatMode vs SetEventFormatCode ")
                return symbol
        return None

    def mark_cmd_tlm_symbols(self):
        '''
        Mark all symbols that are either telemetry or command data types.
        '''
        self._node.get_logger().debug("Marking symbols that are necessary for messages.")
        self._empty_symbols = []
        for key in self._symbol_name_map.keys():
            symbol = self._symbol_name_map[key]
            if symbol.get_is_command() or symbol.get_is_telemetry():
                self.mark_output_symbol(symbol)

    def mark_output_symbol(self, symbol):
        '''
        Mark this symbol as one that should be written to a msg file and mark each of its fields.

        Args:
            symbol (symbol): The symbol to mark
        '''
        symbol.set_should_output(True)
        byte_size = symbol.get_size()
        fields = symbol.get_fields()
        fields.sort(key=lambda field: field._byte_offset)
        prev_field = None
        for field in fields:
            field_symbol = field.get_type_symbol()
            self.mark_output_symbol(field_symbol)
            if prev_field is not None:
                prev_field.set_byte_length(field.get_byte_offset() - prev_field.get_byte_offset())
            prev_field = field
        # need to set last field length
        if prev_field is not None:
            prev_field.set_byte_length(byte_size - prev_field.get_byte_offset())

    def get_symbol_name_map(self):
        '''
        Return the symbol list as a map of symbol name to symbol object.

        Returns:
            symbol_name_map (dict): A dictionary of symbols.
        '''
        return self._symbol_name_map

    def get_field_name_map(self):
        '''
        Return the field list as a map of field name to field object.

        Returns:
            field_name_map (dict): A dictionary of fields.
        '''
        return self._field_name_map

    def handle_duplicate_fields(self):
        '''
        Check all fields for duplicate names and rename as necessary.
        '''
        for symbol_name in self._symbol_name_map:
            symbol = self._symbol_name_map[symbol_name]
            if len(symbol.get_fields()) > 0:
                self.rename_duplicate_fields(symbol)

    def rename_duplicate_fields(self, symbol):
        '''
        Rename all fields that have the given name.

        Args:
            symbol (symbol): A symbol with a name that isn't unique
        '''
        v_names = {}

        fields = symbol.get_fields()
        fields.sort(key=field_byte_order)
        for field in fields:
            # typename = field.get_type_name()
            fn = field.get_ros_name()
            if fn not in v_names.keys():
                v_names[fn] = 0
            else:
                v_names[fn] += 1
                fn = fn + "_" + str(v_names[fn])
                field.update_ros_name(fn)
