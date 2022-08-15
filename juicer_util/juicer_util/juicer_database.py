import sqlite3
from sqlite3 import Error
from juicer_util.juicer_fields import JuicerFieldEntry
from juicer_util.juicer_symbols import JuicerSymbolEntry


class JuicerDatabase():

    def __init__(self, node, db_file):

        self._node = node
        self._node.get_logger().info("Loading message data from Juicer SQLite databases")
        self._db_file = db_file
        self._field_name_map = dict()
        self._symbol_name_map = dict()
        self._symbol_id_map = dict()

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
            self._node.get_logger().error(e)

        return conn

    def retrieve_all_symbols(self):
        """
        Query all rows in the symbols table
        :return: A mapping of symbol name to symbol object
        """
        cur = self._conn.cursor()
        cur.execute("SELECT * FROM symbols")
        rows = cur.fetchall()

        for row in rows:
            my_symbol = JuicerSymbolEntry(self._node, row[0], row[1], row[2], row[3])
            self._symbol_id_map[my_symbol.getId()] = my_symbol
            if not my_symbol.get_name().startswith("_"):
                self._symbol_name_map[my_symbol.get_name()] = my_symbol
        cur.close()
        return self._symbol_id_map

    def retrieve_all_fields(self):
        """
        Query all rows in the fields table
        :return: A mapping of field name to field object
        """
        cur = self._conn.cursor()
        cur.execute("SELECT * FROM fields")

        rows = cur.fetchall()
        for row in rows:
            my_field = JuicerFieldEntry(row[0], row[1], row[2], row[3],
                                        row[4], row[5], row[6], row[7])
            self._field_name_map[my_field.get_name()] = my_field
            symbol = self._symbol_id_map[my_field.getSymbol()]
            if symbol is not None:
                typeid = my_field.get_type()
                my_field.setTypeSymbol(self._symbol_id_map[typeid])
                symbol.addField(my_field)
        cur.close()
        return self._field_name_map

    def load_data(self):
        self._conn = self.create_connection(self._db_file)
        self.retrieve_all_symbols()
        self.retrieve_all_fields()
        self._conn.close()
        self.prune_symbols_and_fields()
        self.mark_cmd_tlm_symbols()

    def prune_symbols_and_fields(self):
        self._node.get_logger().info("Pruning out things that aren't needed.")
        self._empty_symbols = []
        for key in self._symbol_name_map.keys():
            symbol = self._symbol_name_map[key]
            if len(symbol.get_fields()) == 0:
                self._empty_symbols.append(symbol)
                # for some reason some messages need to be added twice,
                # so just automatically do it for all of them
                self._empty_symbols.append(symbol)
        self._node.get_logger().info("There are " + str(len(self._empty_symbols))
                                     + " empty symbols")
        for symbol in self._empty_symbols:
            mn = symbol.get_ros_name()
            # if it starts with lower case then it is ROS2 native type so ignore it
            if mn[0].isupper():
                altSym = self.find_alternative_symbol(symbol)
                if altSym is not None:
                    self._empty_symbols.remove(symbol)
                    symbol.set_alternative(altSym)
                # else:
                # print("Unable to find an alternative for " + symbol.get_name())
        self._node.get_logger().info("There are " + str(len(self._empty_symbols)) +
                                     " empty symbols left after pruning")

    def find_alternative_symbol(self, empty_symbol):
        for key in self._symbol_name_map.keys():
            symbol = self._symbol_name_map[key]
            if empty_symbol.get_name().startswith(symbol.get_name()):
                if not empty_symbol == symbol:
                    if empty_symbol.getSize() == symbol.getSize():
                        return symbol
                    else:
                        self._node.get_logger().warn("Can't replace " + empty_symbol.get_name() +
                                                     " with " + symbol.get_name())
                        self._node.get_logger().warn("wrong size " + str(empty_symbol.getSize()) +
                                                     " vs " + str(symbol.getSize()))
            elif empty_symbol.get_name() == "CFE_EVS_SetEventFormatMode_Payload_t" and \
                    symbol.get_name() == "CFE_EVS_SetEventFormatCode_Payload":
                self._node.get_logger().info("Handling SetEventFormatMode vs SetEventFormatCode ")
                return symbol
        return None

    def mark_cmd_tlm_symbols(self):
        self._node.get_logger().info("Marking symbols that are necessary for messages.")
        self._empty_symbols = []
        for key in self._symbol_name_map.keys():
            symbol = self._symbol_name_map[key]
            if symbol.get_is_command() or symbol.get_is_telemetry():
                self.mark_output_symbol(symbol)

    def mark_output_symbol(self, symbol):
        symbol.set_should_output(True)
        fields = symbol.get_fields()
        for field in fields:
            field_symbol = field.get_type_symbol()
            self.mark_output_symbol(field_symbol)

    def get_symbol_name_map(self):
        return self._symbol_name_map

    def get_field_name_map(self):
        return self._field_name_map
