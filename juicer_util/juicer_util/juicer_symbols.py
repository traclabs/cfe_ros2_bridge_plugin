"""
.. module:: cfe_ros2_bridge_plugin.juicer_util.juicer_symbols
   :synopsis: Class that holds an entry from the symbols table of the juicer database

.. moduleauthor:: Tod Milam

"""


# Class to hold an entry in the symbols table of the database generated by juicer.
class JuicerSymbolEntry():
    """Holds the data from an entry in the symbol table of database, representing a data struct.

    Attributes
    ----------
    node : rosnode
        the ROS2 node
    my_id : str
        unique id for this symbol
    elf : int
        unused column
    name : str
        name of this symbol
    byte_size : int
        size of this data structure
    fields : list
        list of fields this structure contains
    is_command : bool
        if true then this structure represents a command object
    is_telemetry : bool
        if true then this structure represents a telemetry object
    should_output : bool
        if true then this structure should be written to a message file
    ros_name : str
        the generated ROS2 style name for this symbol
    alternative : JuicerSymbolEntry
        the symbol that should be used instead of this symbol

    Methods
    -------
    get_name():
        Return this symbols name.
    set_name(new_name):
        Set this symbols name.
    get_ros_name():
        Returns this symbols ROS2 style name.
    get_ros_name_array():
        Return this symbols ROS2 style name as an array.
    set_ros_name(name):
        Sets this symbols ROS2 style name.
    get_size():
        Return the size in bytes of this symbol.
    get_alternative():
        Returns the preferred alternative symbol to this symbol.
    set_alternative(altern):
        Sets the preferred alternative symbol to this symbol.
    get_ros_topic():
        Returns the ROS2 topic for this symbol.
    get_id():
        Returns the unique id for this symbol.
    add_field(field):
        Adds a field to this symbol.
    get_fields():
        Returns the list of fields for this symbol.
    get_is_command():
        Returns true if this symbol represents a command.
    get_is_telemetry():
        Returns true if this symbol represents a telemetry object.
    get_should_output():
        Returns true if this symbol should be written to a ROS2 message file.
    set_should_output(output):
        Sets if this symbol should be written to a ROS2 message file.
    generate_ros_name(symbol_name):
        Generate a ROS2 style name from the given name.
    handle_lower_case_name(lc_name, symbol_name):
        Helper function for generating ROS2 style name.
    field_byte_order(field):
        Helper function to sort fields by their byte offset.
    """

    def __init__(self, node, my_id, elf, name, byte_size):
        '''
        Initializes the symbol attributes for this object.

        Args:
            node (rosnode): The ROS2 node
            my_id (str): A unique id for this symbol
            elf (int): Unused column
            name (str): The name of this symbol
            byte_size (int): The size of this data structure
        '''
        self._node = node
        self._my_id = my_id
        self._elf = elf
        self._name = name
        self._byte_size = byte_size
        self._fields = []
        # flags indicating if this is a command or telemetry message
        self._is_command = False
        self._is_telemetry = False
        self._should_output = False
        self._ros_name = self.generate_ros_name(self._name)
        if self._ros_name == "Bool":
            # not sure why other checks don't fix this
            self._ros_name = "bool"
        self._alternative = None

    def get_name(self):
        '''
        Return this symbols name.

        Returns:
            name (str): The name for this symbol
        '''
        return self._name

    def set_name(self, new_name):
        '''
        Set this symbols name.

        Args:
            new_name (str): The new name for this symbol
        '''
        self._name = new_name

    def get_ros_name(self):
        '''
        Returns this symbols ROS2 style name.

        Returns:
            ros_name (str): The ROS2 style name for this symbol
        '''
        return self._ros_name

    def get_ros_name_array(self):
        '''
        Return this symbols ROS2 style name as an array.

        Returns:
            array_name (str): The ROS2 style name in array form
        '''
        array_name = self._ros_name + "[]"
        if self._ros_name.startswith("char"):
            array_name = "string"
        elif self._ros_name.startswith("string"):
            array_name = "string"
        return array_name

    def set_ros_name(self, name):
        '''
        Sets this symbols ROS2 style name.

        Args:
            name (str): The new ROS2 style name for this symbol
        '''
        self._ros_name = name

    def get_size(self):
        '''
        Return the size in bytes of this symbol.

        Returns:
            byte_size (int): The size of this symbol in bytes
        '''
        return self._byte_size

    def get_alternative(self):
        '''
        Returns the preferred alternative symbol to this symbol.

        Returns:
            alternative (JuicerSymbolEntry): The preferred alternative symbol
        '''
        return self._alternative

    def set_alternative(self, altern):
        '''
        Sets the preferred alternative symbol to this symbol.

        Args:
            altern (JuicerSymbolEntry): The new preferred alternative symbol
        '''
        self._alternative = altern

    def get_ros_topic(self):
        '''
        Returns the ROS2 topic for this symbol.

        Returns:
            topic (str): The generated topic name for this symbol
        '''
        n = self.get_name()
        n = n.replace("-", "_")
        n = n.replace(" ", "_")
        n = n.replace("__", "_")
        n = n.replace("*", "")
        return n.lower()

    def get_id(self):
        '''
        Returns the unique id for this symbol.

        Returns:
            my_id (str): The unique id for this symbol
        '''
        return self._my_id

    def add_field(self, field):
        '''
        Adds a field to this symbol.

        Args:
            field (JuicerFieldEntry): the field to add
        '''
        if not field.get_ros_name():
            t = str(field.get_type())
            self._node.get_logger().debug("Skipping field " + field.get_name() + ", " + t)
        else:
            self._fields.append(field)
            self._fields.sort(key=field_byte_order)
            field_type = field.get_type_name()
            if "TelemetryHeader" in field_type:
                self._is_telemetry = True
            elif "CommandHeader" in field_type:
                self._is_command = True

    def get_fields(self):
        '''
        Returns the list of fields for this symbol.

        Returns:
            fields (list): The list of fields for this symbol
        '''
        return self._fields

    def get_is_command(self):
        '''
        Returns true if this symbol represents a command.

        Returns:
            is_command (bool): If this is a command
        '''
        return self._is_command

    def get_is_telemetry(self):
        '''
        Returns true if this symbol represents a telemetry object.

        Returns:
            is_telemetry (bool): If this is a telemetry object
        '''
        return self._is_telemetry

    def get_should_output(self):
        '''
        Returns true if this symbol should be written to a ROS2 message file.

        Returns:
            should_output (bool): If this should be output to file
        '''
        return self._should_output

    def set_should_output(self, output):
        '''
        Sets if this symbol should be written to a ROS2 message file.

        Args:
            output (bool): If this should be output to file
        '''
        self._should_output = output

    def generate_ros_name(self, symbol_name):
        '''
        Generate a ROS2 style name from the given name.

        Args:
            symbol_name (str): The name used to generate the ROS2 name

        Returns:
            ros_name (str): The generated name
        '''
        n = symbol_name
        if n.endswith('*'):
            # indicates a pointer of size 8 bytes
            n = 'uint64'
        elif symbol_name == "CFE_ResourceId_t":
            n = 'uint32'
        elif symbol_name == "CFE_ES_MemOffset_t":
            n = 'uint32'
        elif symbol_name == "CFE_ES_MemAddress_t":
            n = 'uint32'
        elif symbol_name == "CFE_ES_TaskId_t":
            n = 'uint32'
        elif symbol_name == "CFE_ES_AppId_t":
            n = 'uint32'
        elif symbol_name == "CFE_SB_PipeId_t":
            n = 'uint32'
        elif symbol_name == "CFE_ES_MemHandle_t":
            # not sure what this should be, but this is correct size
            n = 'uint32'
        elif symbol_name == "CFE_ES_ExceptionAction_Enum_t":
            # not sure what this should be, but this is correct size
            n = 'uint8'
        elif symbol_name == "CFE_ES_TaskPriority_Atom_t":
            # not sure what this should be, but this is correct size
            n = 'uint16'
        elif symbol_name == "CFE_SB_MsgId_Atom_t":
            # not sure what this should be, but this is correct size
            n = 'uint32'
        elif symbol_name == "CFE_EVS_LogMode_Enum_t":
            # not sure what this should be, but this is correct size
            n = 'uint8'
        elif symbol_name == "CFE_EVS_MsgFormat_Enum_t":
            # not sure what this should be, but this is correct size
            n = 'uint8'
        elif symbol_name == "CFE_TIME_ClockState_Enum_t":
            # not sure what this should be, but this is correct size
            n = 'uint16'
        n = n.replace("_", "")
        n = n.replace(" ", "")
        n = n.replace("*", "")

        if n[0].islower():
            n = self.handle_lower_case_name(n, symbol_name)

        return n

    def handle_lower_case_name(self, lc_name, symbol_name):
        '''
        Helper function for generating ROS2 style name.

        Args:
            lc_name (str): The lower case name
            symbol_name (str): The symbol name

        Returns:
            name (str): The updated name
        '''
        n = lc_name
        # map simple types to ROS2 types
        if n == 'char':  # indicates a char array, so make it a string
            n = 'string'
        elif n.startswith('uint64'):
            n = 'uint64'
        elif n.startswith('uint32'):
            n = 'uint32'
        elif n.startswith('uint16'):
            n = 'uint16'
        elif n.startswith('uint8'):
            n = 'uint8'
        elif n.startswith('int64'):
            n = 'int64'
        elif n.startswith('int32'):
            n = 'int32'
        elif n.startswith('int16'):
            n = 'int16'
        elif n.startswith('int8'):
            n = 'int8'
        elif n == 'float':
            n = 'float32'
        elif n == 'double':
            n = 'float64'
        elif n == 'padding8':
            n = 'uint8'
        elif n == 'padding16':
            n = 'uint16'
        elif n == 'padding24':
            n = 'string'
        elif n == 'padding32':
            n = 'uint32'
        elif n == 'padding64':
            n = 'uint64'
        elif n.startswith('padding'):
            # how to handle this?
            # print("Setting " + n + " to char")
            n = 'char'
        else:
            n = n.capitalize()
            # print("Capitalized " + n)
        return n


def field_byte_order(field):
    '''
    Helper function to sort fields by their byte offset.

    Args:
        field (JuicerFieldEntry): The field being sorted

    Returns:
        byte_offset (int): The byte offset of the field
    '''
    return field.get_byte_offset()
