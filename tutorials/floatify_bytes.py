import struct
from bitarray import bitarray

# use this function for the recruit induction task (or write your own! this seems like a very messy way of doing it)
# used to convert raw bytes from the data list into floats
def floatify_bytes(z_bytes):
    """
    :param lst of four bytes as numbers between 0-255
    :return: the little endian float representation
    """
    bits = bin(z_bytes[3])[2:].zfill(8) + bin(z_bytes[2])[2:].zfill(8) + bin(z_bytes[1])[2:].zfill(8) + bin(z_bytes[0])[2:].zfill(8)
    arr = bitarray(bits)
    return struct.unpack(">f", arr)[0]


