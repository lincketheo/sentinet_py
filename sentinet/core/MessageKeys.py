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
INT64 = 't' 
FLOAT = 'f' 
DOUBLE = 'd' 
STRING = 's' 
OBJECT = 'o' 
ARRAY = 'a' 

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

