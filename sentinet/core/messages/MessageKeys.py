import struct
# A place for message keys


# The protocl version
PROTOCOL = 1 

# OPCODE FLAGS
DATA_PACKET = 1 
PING = 2 

# Initialized constants
EMPTY = 0 
ENDIAN = 'big'
  
# Type codes
INVALID = '\0'
CHAR = 'c' 
BYTE_TYPE = 'y' 
BOOL = 'b' 
INT8 = 'p' 
UINT8 = 'r' 
UINT16 = 'q' 
INT32 = 'i' 
INT = INT32
UINT32 = 'u' 
INT64 = 'x'
UINT64 = 't' 
FLOAT = 'f' 
DOUBLE = 'd' 
STRING = 's' 
OBJECT = 'o' 
ARRAY = 'a' 


# I had a lot of fun with lambdas
# Each element has an assert and a serialize callback
# 1st lambda checks that the data is valid, second returns a byte version of that data
serialize_funcs = {
        INVALID :   ((lambda val, byte_length: False), 
                    (lambda val: False)),

        CHAR :      ((lambda val, byte_length: byte_length == 1 and type(val) == str and len(val) == 1), 
                    (lambda val: ord(val))),

        BYTE_TYPE : ((lambda val, byte_length: byte_length == 1 and type(val) == bytes and len(val) == 1), 
                    (lambda val: val)),

        BOOL :      ((lambda val, byte_length: byte_length == 1 and type(val) == bool), 
                    (lambda val: 1 if val else 0)),

        INT8 :      ((lambda val, byte_length: byte_length == 1 and type(val) == int and val < 128 and val > -129),
                    (lambda val : val.to_bytes(1, byteorder = ENDIAN, signed = True))),

        UINT8 :     ((lambda val, byte_length: byte_length == 1 and type(val) == int and val < 256),
                    (lambda val : val.to_bytes(1, byteorder = ENDIAN, signed = False))),

        UINT16 :    ((lambda val, byte_length: byte_length == 2 and type(val) == int and val < 65536),
                    (lambda val : val.to_bytes(2, byteorder = ENDIAN, signed = False))),

        INT32 :     ((lambda val, byte_length: byte_length == 4 and type(val) == int and val < 2147483648 and val > -2147483649),
                    (lambda val : val.to_bytes(4, byteorder = ENDIAN, signed = True))),

        UINT32 :    ((lambda val, byte_length: byte_length == 4 and type(val) == int and val < 4294967296),
                    (lambda val : val.to_bytes(4, byteorder = ENDIAN, signed = False))),

        INT64 :     ((lambda val, byte_length: byte_length == 8 and type(val) == int and val < 9223372036854775808 and val > -9223372036854775809), 
                    (lambda val : val.to_bytes(8, byteorder = ENDIAN, signed = True))),

        UINT64 :    ((lambda val, byte_length: byte_length == 8 and type(val) == int and val < 18446744073709551616), 
                    (lambda val : val.to_bytes(8, byteorder = ENDIAN, signed = False))),

        FLOAT :     ((lambda val, byte_length: byte_length == 4 and type(val) == float), 
                    (lambda val : struct.pack("f", val))),

        DOUBLE :    ((lambda val, byte_length: byte_length == 8 and type(val) == float), 
                    (lambda val : struct.pack("d", val))),

        STRING :    ((lambda val, byte_length: type(val) == str and len(val) + 1 == byte_length),
                    (lambda val : val.encode('utf-8') + b'\x00')),

        OBJECT :    ((lambda val, byte_length: False),
                    (lambda val : print("Idiot, don't serielaize objects"))),

        ARRAY :     ((lambda val, byte_length: type(val) == bytearray and len(val) == byte_length),
                    (lambda val : val))}

# Stores and reivieves the index and values of a data element setting data size, the fact that it is positive or negative where it is and its value.
class DataElement:
    def __init__(self, value, data_size, index, signed):
        self.data_size = data_size
        self.signed = signed
        self.data = value
        self.index = index

    def serialize(self) -> bytes:
        return self.data.to_bytes(self.data_size, byteorder = ENDIAN, signed = self.signed)

default_header = {
      "PROTOCOL" : DataElement(PROTOCOL, 1, 0, False),
      "OPCODE" : DataElement(DATA_PACKET, 1, 1, False),
      "FUNCFLAGS" : DataElement(EMPTY, 2, 2, False),
      "CHECKSUM" : DataElement(EMPTY, 2, 4, False),
      "BYTE_LENGTH" : DataElement(13, 4, 6, False),
      "FIELDS" : DataElement(EMPTY, 2, 10, False) }

default_ping = {
      "PROTOCOL" : DataElement(PROTOCOL, 1, 0, False),
      "OPCODE" : DataElement(PING, 1, 1, False),
      "TYPE" : DataElement(EMPTY, 1, 2, False),
      "CODE" : DataElement(EMPTY, 2, 3, False),
      "CHECKSUM" : DataElement(EMPTY, 2, 5, False),
      "EXCESS" : DataElement(EMPTY, 8, 7, False) }

