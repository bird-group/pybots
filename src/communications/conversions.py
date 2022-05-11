
from struct import unpack
from struct import pack
from numbers import Number
import pdb

import numpy

def bytes_to_u16(byte_array):
    """ Unpack an unsigned 16 bit int from a byte array

    byte_array must have a length which is 0 modulo 2 (ie an integer
    number of uint16's)

    Arguments
        byte_array: a byte array to unpack

    Returns:
        u16_array: a list of u16's made from byte_array
    """
    assert len(byte_array) % 2 == 0, "byte_array must have an integer\
        number of uint16's in it"

    if type(byte_array) is list:
        byte_array = ''.join(byte_array)

    fmt = 'H'*(len(byte_array)/2)
    return unpack(fmt, byte_array)

def bytes_to_i16(byte_array):
    """ Unpack an signed 16 bit int from a byte array

    byte_array must have a length which is 0 modulo 2 (ie an integer
    number of int16's)

    Arguments
        byte_array: a byte array to unpack

    Returns:
        i16_array: a list of i16's made from byte_array
    """
    assert len(byte_array) % 2 == 0, "byte_array must have an integer\
        number of int16's in it"

    if type(byte_array) is list:
        byte_array = ''.join(byte_array)

    fmt = 'h'*(len(byte_array)/2)
    return unpack(fmt, byte_array)

def bytes_to_u24(byte_array):
    """ Unpack an unsigned 24 bit int from a byte array

    byte_array must have a length which is 0 modulo 3 (ie an integer
    number of uint24's)

    Arguments
        byte_array: a byte array to unpack

    Returns:
        u24_array: a list of u24's made from byte_array
    """
    assert len(byte_array) % 3 == 0, "byte_array must have an integer\
        number of uint24's in it"

    if type(byte_array) is list:
        byte_array = ''.join(byte_array)

    expanded_array = ''
    for i, b in zip(range(len(byte_array)), byte_array):
        expanded_array += b
        if i%3 == 2:
            expanded_array += '\x00'

    fmt = 'I'*(len(byte_array)/3)
    return unpack(fmt, expanded_array)

def bytes_to_i24(byte_array):
    """ Unpack an unsigned 24 bit int from a byte array

    byte_array must have a length which is 0 modulo 3 (ie an integer
    number of uint24's)

    Arguments
        byte_array: a byte array to unpack

    Returns:
        u24_array: a list of u24's made from byte_array
    """
    assert len(byte_array) % 3 == 0, "byte_array must have an integer\
        number of uint24's in it"

    if type(byte_array) is list:
        byte_array = ''.join(byte_array)

    expanded_array = ''
    for i, b in zip(range(len(byte_array)), byte_array):
        expanded_array += b
        if i%3 == 2:
            if unpack('b', b)[0] > 0:
                expanded_array += '\x00'
            else:
                expanded_array += '\xff'

    fmt = 'i'*(len(byte_array)/3)
    return unpack(fmt, expanded_array)

def bytes_to_u32(byte_array):
    """ Unpack an unsigned 32 bit int from a byte array

    byte_array must have a length which is 0 modulo 4 (ie an integer
    number of uint32's)

    Arguments
        byte_array: a byte array to unpack

    Returns:
        u32_array: a list of u32's made from byte_array
    """
    assert len(byte_array) % 4 == 0, "byte_array must have an integer\
        number of uint32's in it"

    if type(byte_array) is list:
        byte_array = ''.join(byte_array)

    fmt = 'I'*(len(byte_array)/4)
    return unpack(fmt, byte_array)

def bytes_to_i32(byte_array):
    """ Unpack a signed 32 bit int from a byte array

    byte_array must have a length which is 0 modulo 4 (ie an integer
    number of uint32's)

    Arguments
        byte_array: a byte array to unpack

    Returns:
        i32_array: a list of i32's made from byte_array
    """
    assert len(byte_array) % 4 == 0, "byte_array must have an integer\
        number of int32's in it"

    if type(byte_array) is list:
        byte_array = ''.join(byte_array)

    fmt = 'i'*(len(byte_array)/4)
    return unpack(fmt, byte_array)

def bytes_to_float32(byte_array):
    """ Unpack a 32 bit float from a byte array

    byte_array must have a length which is 0 modulo 4 (ie an integer
    number of floats's)

    Arguments
        byte_array: a byte array to unpack

    Returns:
        float_array: a list of floats made from byte_array
    """
    assert len(byte_array) % 4 == 0, "byte_array must have an integer\
        number of floats in it"

    if type(byte_array) is list:
        byte_array = ''.join(byte_array)

    fmt = 'f'*(len(byte_array)/4)
    return unpack(fmt, byte_array)

def bytes_to_float64(byte_array):
    """ Unpack a 64 bit float from a byte array

    byte_array must have a length which is 0 modulo 8 (ie an integer
    number of doubles's)

    Arguments
        byte_array: a byte array to unpack

    Returns:
        double_array: a list of doubles made from byte_array
    """
    assert len(byte_array) % 8 == 0, "byte_array must have an integer\
        number of doubles in it"

    if type(byte_array) is list:
        byte_array = ''.join(byte_array)

    fmt = 'd'*(len(byte_array)/8)
    return unpack(fmt, byte_array)

def u16_to_bytes(u16):
    """ Pack an unsigned 16 bit int into a byte array

    Arguments:
        u16: a list of u16's to pack

    Returns:
        byte_array: a byte array to pack
    """
    if type(u16) is list:
        byte_array = ''
        for u in u16:
            byte_array += u16_to_bytes(u)
        return byte_array

    assert isinstance(u16, Number), "input must be a number or a list\
        of numbers"
    assert u16 < 65536, "input must be less than 65536 for uint16"
    assert u16 > 0, "input must greater than 0 for uint16"
    return pack('H', u16)

def i16_to_bytes(i16):
    """ Pack a signed 16 bit int into a byte array

    Arguments:
        i16: a list of i16's to pack

    Returns:
        byte_array: a byte array to pack
    """
    if type(i16) is list:
        byte_array = ''
        for i in i16:
            byte_array += i16_to_bytes(i)
        return byte_array

    assert isinstance(i16, Number), "input must be a number or a list\
        of numbers"
    assert i16 < 32767, "input must be less than 32767 for int16"
    assert i16 > -32768, "input must be greater than -32768 for int16"
    return pack('h', i16)

def u24_to_bytes(u24):
    """ Pack an unsigned 24 bit int into a byte array

    Arguments
        u24: a number or list of them to pack

    Returns:
        byte_array: a byte representing the u24s
    """
    if type(u24) is list:
        byte_array = ''
        for u in u24:
            byte_array += u24_to_bytes(u)
        return byte_array

    assert isinstance(u24, Number), "input must be a number or a list\
        of numbers"
    assert u24 < 16777216, "u24 must be less than 16777216"
    assert u24 > 0, "u24 must be greater than 0"
    return pack('I', u24)[0:-1]

def i24_to_bytes(i24):
    """ Pack a signed 24 bit int into a byte array

    Arguments
        i24: a number or list of them to pack

    Returns:
        byte_array: a byte representing the i24s
    """
    if type(i24) is list:
        byte_array = ''
        for i in i24:
            byte_array += i24_to_bytes(i)
        return byte_array

    assert isinstance(i24, Number), "input must be a number or a list\
        of numbers"
    assert i24 < 16777216, "u24 must be less than 8388608"
    assert i24 > -8388608, "u24 must be greater than -8388608"
    return pack('i', i24)[0:-1]

def u32_to_bytes(u32):
    """ Pack an unsigned 32 bit int into a byte array

    Arguments:
        u32_array: a list of u32's to pack

    Returns:
        byte_array: a byte array to pack
    """
    if type(u32) is list:
        byte_array = ''
        for u in u32:
            byte_array += u32_to_bytes(u)
        return byte_array

    assert isinstance(u32, Number), "input must be a number or a list\
        of numbers"
    assert u32 < 4294967296, "u32 must be less than 4294967296"
    assert u32 > 0, "u24 must be greater than 0"
    return pack('I', u32)

def i32_to_bytes(i32):
    """ Pack an signed 32 bit int into a byte array

    Arguments:
        i32_array: a list of i32's to pack

    Returns:
        byte_array: a byte array to pack
    """
    if type(i32) is list:
        byte_array = ''
        for i in i32:
            byte_array += i32_to_bytes(i)
        return byte_array

    assert isinstance(i32, Number), "input must be a number or a list\
        of numbers"
    assert i24 < 2147483648, "u24 must be less than 2147483648"
    assert i24 > -2147483649, "u24 must be greater than -2147483649"
    return pack('i', i32)

def float32_to_bytes(f32):
    """ Pack a 32 bit float into a byte array

    Arguments:
        f32: a list of floats to pack

    Returns:
        byte_array: a byte array to pack
    """
    if type(f32) is list:
        byte_array = ''
        for f in f32:
            byte_array += float32_to_bytes(f)
        return byte_array

    assert isinstance(f32, Number), "input must be a number or a list\
        of numbers"
    return pack('f', f32)

def float64_to_bytes(f64):
    """ Pack a 64 bit float into a byte array

    Arguments:
        f64: a list of floats to pack

    Returns:
        byte_array: a byte array to pack
    """
    if type(f64) is list:
        byte_array = ''
        for f in f64:
            byte_array += float64_to_bytes(f)
        return byte_array

    assert isinstance(f64, Number), "input must be a number or a list\
        of numbers"
    return pack('d', f64)


