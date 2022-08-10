

#!/usr/bin/env python3

# Class to hold an entry in the symbols table of the database generated by juicer.
class JuicerSymbolEntry():

    def __init__(self, my_id, elf, name, byte_size):
        self._my_id = my_id
        self._elf = elf
        self._name = name
        self._byte_size = byte_size
        self._fields = []
        # flags indicating if this is a command or telemetry message
        self._is_command = False
        self._is_telemetry = False
        self._should_output = False
        self._ros_rame = generate_ros_rame(self._name)
        if self._ros_rame == "Bool":
            # not sure why other checks don't fix this
            self._ros_rame = "bool"
        self._alternative = None

    def get_name(self):
        return self._name

    def get_ros_name(self):
        return self._ros_rame

    def get_size(self):
        return self._byte_size

    def get_alternative(self):
        return self._alternative

    def set_alternative(self, altern):
        self._alternative = altern

    def get_ros_topic(self):
        n = self.get_name()
        n = n.replace("-", "_")
        n = n.replace(" ", "_")
        n = n.replace("__", "_")
        n = n.replace("*", "")
        return n.lower()

    def get_id(self):
        return self._my_id

    def add_field(self, field):
        if not field.get_ros_name():
            print("Skipping field " + field.get_name() + " of type " + str(field.get_type()))
        else:
            self._fields.append(field)
            field_type = field.get_type_name()
            if "TelemetryHeader" in field_type:
                self._is_telemetry = True
            elif "CommandHeader" in field_type:
                self._is_command = True

    def get_fields(self):
        return self._fields

    def get_is_command(self):
        return self._is_command

    def get_is_telemetry(self):
        return self._is_telemetry

    def get_ghould_output(self):
        return self._should_output

    def set_should_output(self, output):
        self._should_output = output


def generate_ros_rame(symbol_name):
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
        n = handle_lower_case_name(n, symbol_name)

    return n


def handle_lower_case_name(lc_name, symbol_name):
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
    elif n == 'padding8':
        n = 'uint8'
    elif n == 'padding16':
        n = 'uint16'
    elif n == 'padding32':
        n = 'uint32'
    elif n == 'padding64':
        n = 'uint64'
    elif n.startswith('padding'):
        # how to handle this?
        #print("Setting " + n + " to char")
        n = 'char'
    else:
        n = n.capitalize()
        #print("Capitalized " + n)
    return n
