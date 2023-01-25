#!/usr/bin/env python3
import re


# Class to hold an entry in the fields table of the database generated by juicer.
class JuicerFieldEntry():

    def __init__(self, node, my_id, symbol, name, byte_offset, my_type,
                 little_endian, bit_size, bit_offset):
        self._node = node
        self._my_id = my_id
        self._symbol = symbol
        self._name = name
        self._byte_offset = byte_offset
        self._my_type = my_type
        self._little_endian = little_endian
        self._bit_size = bit_size
        self._bit_offset = bit_offset
        self._ros_name = generate_ros_field_name(name)
        self._type_symbol = None
        self._byte_length = 0

    def get_name(self):
        return self._name

    def get_symbol(self):
        return self._symbol

    def get_id(self):
        return self._my_id

    def get_type(self):
        return self._my_type

    def get_ros_name(self):
        return self._ros_name

    def get_byte_offset(self):
        return self._byte_offset

    def get_bit_offset(self):
        return self._bit_offset

    def get_type_name(self):
        is_array, length = self.get_is_array()
        symbol = self.get_type_symbol()
        if is_array:
            type_name = symbol.get_ros_name_array()
            self._node.get_logger().info("Found array for " + symbol.get_ros_name() + " of length " + str(length))
        else:
            type_name = symbol.get_ros_name()
        return type_name

    def get_is_array(self):
        retval = False
        symbol = self.get_type_symbol()
        # determine if this is an array
        size = symbol.get_size()
        length = self._byte_length / size
        if length >= 2:
            retval = True
        else:
            length = 1
        return retval, length

    def set_type_symbol(self, symbol):
        self._type_symbol = symbol

    def get_type_symbol(self):
        symbol = self._type_symbol
        if symbol.get_alternative():
            symbol = symbol.get_alternative()
        return symbol

    def get_endian(self):
        return self._little_endian

    def set_little_endian(self, endian):
        self._little_endian = endian

    def get_byte_length(self):
        return self._byte_length

    def set_byte_length(self, length):
        self._byte_length = length


def generate_ros_field_name(name):
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

    return replace_keywords(retval)


def replace_keywords(name):
    if name == "virtual":
        return "virt"
    return name
