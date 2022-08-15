#!/usr/bin/env python3
import re


# Class to hold an entry in the fields table of the database generated by juicer.
class JuicerFieldEntry():

    def __init__(self, my_id, symbol, name, byte_offset, my_type,
                 little_endian, bit_size, bit_offset):
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
        symbol = self._type_symbol
        if symbol.get_alternative():
            symbol = symbol.get_alternative()
        return symbol.get_ros_name()

    def set_type_symbol(self, symbol):
        self._type_symbol = symbol

    def get_type_symbol(self):
        symbol = self._type_symbol
        if symbol.get_alternative():
            symbol = symbol.get_alternative()
        return symbol


def generate_ros_field_name(name):
    retval = name

    if any(ele.isupper() for ele in name):
        n = name
        # handle strings of caps - change them to one cap followed by lowercase
        # so that it will be one "word" in final result
        def callback(m): m.group(0).capitalize()
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

    return retval
