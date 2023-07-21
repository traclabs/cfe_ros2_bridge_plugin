"""
.. module:: cfe_ros2_bridge_plugin.juicer_util.juicer_fields
   :synopsis: Class that holds an entry from the fields table of the juicer database

.. moduleauthor:: Tod Milam

"""

import re


class JuicerFieldEntry():
    """This class holds the data from an entry in the field table of the database.

    Attributes
    ----------
    node : rosnode
        the ROS2 node
    my_id : str
        a unique id for this field
    symbol : int
        the id of the symbol that contains this field
    name : str
        the name of this field
    byte_offset : int
        the byte offset for this field in the structure
    my_type : int
        the id of the symbol for the data type of this field
    little_endian : boolean
        is this data stored as little endian
    bit_size : int
        field from database that is not used
    bit_offset : int
        field from database that is not used
    ros_name : str
        the generated ROS2 style name for this field
    type_symbol : symbol
        the symbol for the data type of this field
    byte_length : int
        the length in bytes of this field, used to determine if this is an array

    Methods
    -------
    get_name():
        Return the name of this field.
    get_symbol():
        Return the id of the symbol that contains this field.
    get_id():
        Return the unique id of this field.
    get_type():
        Return the id of the symbol for the data type of this field.
    get_ros_name():
        Return the ROS2 style name for this field.
    get_byte_offset():
        Return the byte offset for this field.
    get_type_name():
        Return the ROS2 name for this field as used in a msg file.
    get_is_array():
        Return if this field is an array and its length.
    set_type_symbol(symbol):
        Sets type_symbol to symbol.
    get_type_symbol():
        Returns the symbol type of this field.
    get_endian():
        Returns TRUE if this field is little endian.
    set_little_endian(endian):
        Sets the endian value for this field.
    get_byte_length():
        Return the byte length of this field.
    set_byte_length(length):
        Set the byte length of this field.
    update_ros_name(name):
        Set the ROS2 name of this field to name.
    generate_ros_field_name(name):
        Generate a ROS2 style field name based on input name.
    replace_keywords(name):
        Replace common keywords with ROS2 style versions in name.
    """

    def __init__(self, node, my_id, symbol, name, byte_offset, my_type,
                 little_endian, bit_size, bit_offset):
        '''
        Initializes the field attributes for this object.

            Parameters:
                    node (rosnode): The ROS2 node
                    my_id (str): A unique id for this field
                    symbol (int): The id of the symbol that contains this field
                    name (str): The name of this field
                    byte_offset (int): The byte offset for this field in the structure
                    my_type (int): The id of the symbol for the data type of this field
                    little_endian (bool): Is this data stored as little endian
                    bit_size (int): Field from database that is not used
                    bit_offset (int): Field from database that is not used
        '''
        self._node = node
        self._my_id = my_id
        self._symbol = symbol
        self._name = name
        self._byte_offset = byte_offset
        self._my_type = my_type
        self._little_endian = little_endian
        self._bit_size = bit_size
        self._bit_offset = bit_offset
        self._ros_name = self.generate_ros_field_name(name)
        self._type_symbol = None
        self._byte_length = 0

    def get_name(self):
        '''
        Return the name of this field.

            Returns:
                    name (str): The name of this field
        '''
        return self._name

    def get_symbol(self):
        '''
        Return the id of the symbol that contains this field.

            Returns:
                    symbol (int): The id of the symol that contains this field
        '''
        return self._symbol

    def get_id(self):
        '''
        Return the unique id of this field.

            Returns:
                    my_id (str): The unique id of this field
        '''
        return self._my_id

    def get_type(self):
        '''
        Return the id of the symbol for the data type of this field.

            Returns:
                    my_type (int): The id of the symbol for the data type of this field
        '''
        return self._my_type

    def get_ros_name(self):
        '''
        Return the ROS2 style name for this field.

            Returns:
                    ros_name (str): The ROS2 style name for this field
        '''
        return self._ros_name

    def get_byte_offset(self):
        '''
        Return the byte offset for this field.

            Returns:
                    byte_offset (int): The byte offset for this field
        '''
        return self._byte_offset

    def get_type_name(self):
        '''
        Return the ROS2 name for this field as used in a msg file.

            Returns:
                    type_name (str): The ROS2 name for this field
        '''
        is_array, length = self.get_is_array()
        symbol = self.get_type_symbol()
        if is_array:
            type_name = symbol.get_ros_name_array()
            self._node.get_logger().debug("Found array for " + symbol.get_ros_name()
                                          + " of length " + str(length) + " for field "
                                          + self._ros_name)
        else:
            type_name = symbol.get_ros_name()
        return type_name

    def get_is_array(self):
        '''
        Return if this field is an array and its length.

            Returns:
                    is_array (bool): If this field is an array
                    length (int): The length of the array
        '''
        retval = False
        symbol = self.get_type_symbol()
        # determine if this is an array
        size = symbol.get_size()
        length = int(self._byte_length / size)
        if length >= 2:
            retval = True
        else:
            length = 1
        return retval, length

    def set_type_symbol(self, symbol):
        '''
        Sets type_symbol to symbol.

            Parameters:
                    symbol (symbol): The symbol for the data type of this field
        '''
        self._type_symbol = symbol

    def get_type_symbol(self):
        '''
        Returns the symbol type of this field.

            Returns:
                    type_symbol (symbol): The symbol for the data type of this field
        '''
        symbol = self._type_symbol
        if symbol.get_alternative():
            symbol = symbol.get_alternative()
        return symbol

    def get_endian(self):
        '''
        Returns TRUE if this field is little endian.

            Returns:
                    little_endian (bool): If this field is little endian
        '''
        return self._little_endian

    def set_little_endian(self, endian):
        '''
        Sets the endian value for this field.

            Parameters:
                    endian (bool): True if this field is little endian
        '''
        self._little_endian = endian

    def get_byte_length(self):
        '''
        Return the byte length of this field.

            Returns:
                    byte_length (int): The length in bytes of this field
        '''
        return self._byte_length

    def set_byte_length(self, length):
        '''
        Set the byte length of this field.

            Parameters:
                    length (int): The length in bytes of this field
        '''
        self._byte_length = length

    def update_ros_name(self, name):
        '''
        Set the ROS2 name of this field to name.

            Parameters:
                    name (str): The ROS2 name of this field
        '''
        self._ros_name = name


    def generate_ros_field_name(self, name):
        '''
        Generate a ROS2 style field name based on input name.

            Parameters:
                    name (str): The cFS name of the field

            Returns:
                    ros_name (str): The ROS2 style name of the field
        '''
        retval = name

        if any(ele.isupper() for ele in name):
            n = name

            # handle strings of caps - change them to one cap followed by lowercase
            # so that it will be one "word" in final result
            def callback(m):
                return m.group(0).capitalize()
            n = re.sub(r'[A-Z](?:[A-Z]*(?![a-z]))', callback, n)
            # split into list of words by capital letters
            w = re.findall('([A-Z][a-z]*)', n)
            # combine all words with underscore between them
            retval = '_'.join(w)
            # return value as lower case string
            retval = retval.lower()
        else:
            n = name
            if n.startswith("_"):
                retval = n.strip("_")

        # some special cases that have cropped up
        if name == 'dest_IP':
            retval = 'dest_ip'
        elif name == 'tlm_dest_IP':
            retval = 'tlm_dest_ip'
        elif name == 'CCSDS':
            retval = 'ccsds'

        return self.replace_keywords(retval)


    def replace_keywords(self, name):
        '''
        Replace common keywords with ROS2 style versions in name.

            Parameters:
                    name (str): A keyword for possible replacement

            Returns:
                    name (str): The possibly replaced keyword
        '''
        if name == "virtual":
            return "virt"
        return name
