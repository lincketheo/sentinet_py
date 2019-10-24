# TODO - I put a lot of numbers here, and that's bad practice
# We need to replace all numerical constants in a way that we 
# can update the protocol version whilst being able to change the 
# message structure and individual element size


# Some global vars - might want a keys file
# Like I did with c
PROTOCOL = 1
DATA_PACKET = 1
PING = 2
EMPTY = 0
ENDIAN = 'big'
BYTE_SIZE = 4096

# Type codes - might be a better way of doing this
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

# A single data element 
# This is more of a constant data element
# within a serialized item
class DataElement:
    def __init__(self, value, data_size, index, signed):
        self.data_size = data_size
        self.signed = signed
        self.data = value
        self.index = index

    def serialize(self) -> bytes:
        return self.data.to_bytes(self.data_size, byteorder = ENDIAN, signed = self.signed)


class Data_Message:

    def __init__(self, incomming_message=0): 
        if incomming_message is not 0:
            self.message = incomming_message
            # TODO This looks pretty ugly, 
            # considering get_header... passes the same numbers as DataElement (1, 0 == 1, 0)
            self.header = {
                    "PROTOCOL" : DataElement(self.get_header_element_raw(1, 0), 1, 0, False),
                    "OPCODE" : DataElement(self.get_header_element_raw(1, 1), 1, 1, False),
                    "FUNCFLAGS" : DataElement(self.get_header_element_raw(2, 2), 2, 2, False),
                    "CHECKSUM" : DataElement(self.get_header_element_raw(2, 4), 2, 4, False),
                    "BYTE_LENGTH" : DataElement(self.get_header_element_raw(4, 6), 4, 6, False),
                    "FIELDS" : DataElement(self.get_header_element_raw(2, 10), 2, 10, False) }
        else:
            self.header = {
                    "PROTOCOL" : DataElement(PROTOCOL, 1, 0, False),
                    "OPCODE" : DataElement(DATA_PACKET, 1, 1, False),
                    "FUNCFLAGS" : DataElement(EMPTY, 2, 2, False),
                    "CHECKSUM" : DataElement(EMPTY, 2, 4, False),
                    "BYTE_LENGTH" : DataElement(13, 4, 6, False),
                    "FIELDS" : DataElement(EMPTY, 2, 10, False) }

            self.message = bytearray(13) 

            self.serialize_header()

    # Emplaces all header elements in self.header
    # into the message buffer
    def serialize_header(self):
        i = 0
        for element in self.header.values():
            self.message[i:i+element.data_size] = element.serialize()
            i += element.data_size
        self.message[i] = 0
         
    # Returns element at the header
    def get_header_element(self, element):
        if element in self.header:
            return self.header[element].data
        else:
            print("Invalid elemet")

    def get_header_element_raw(self, size, index):
        return int.from_bytes(self.message[index:index+size], byteorder = ENDIAN, signed=False)

    def serialize_data(self, data, bytes_size, type_code):
        # Check data type, otherwise, we
        # simply represent it as an int
        if(type(data) == str):
            if(bytes_size != len(data) + 1):
                print("ERROR")
                return 0
            # Add a string null terminator
            data = data.encode('utf-8') + b"\x00"
            bytes_size
        else:
            # Convert data to bytes
            data = data.to_bytes(bytes_size, byteorder = ENDIAN, signed = False)

        # Append data
        self.message.append(bytes_size)
        self.message.append(ord(type_code))
        self.message.extend(data)
        self.message.append(0)

        # Update header fields
        self.header["FIELDS"].data += 1
        self.header["BYTE_LENGTH"].data += 3 + bytes_size

class Ping_Message:

    def __init__(self, incomming_message = 0):
        if incomming_message == 0:
            self.header = {
                    "PROTOCOL" : DataElement(PROTOCOL, 1, 0, False),
                    "OPCODE" : DataElement(PING, 1, 1, False),
                    "TYPE" : DataElement(EMPTY, 1, 2, False),
                    "CODE" : DataElement(EMPTY, 2, 3, False),
                    "CHECKSUM" : DataElement(EMPTY, 2, 5, False),
                    "EXCESS" : DataElement(EMPTY, 8, 7, False) }
            self.message = bytearray(16)
        else: 
            self.message = incomming_message
            self.header = {
                    "PROTOCOL" : DataElement(self.get_element_raw(1, 0), 1, 0, False),
                    "OPCODE" : DataElement(self.get_element_raw(1, 1), 1, 1, False),
                    "TYPE" : DataElement(self.get_element_raw(1, 2), 1, 2, False),
                    "CODE" : DataElement(self.get_element_raw(2, 3), 2, 3, False),
                    "CHECKSUM" : DataElement(self.get_element_raw(2, 5), 2, 5, False),
                    "EXCESS" : DataElement(self.get_element_raw(8, 7), 8, 7, False) }


    def serialize(self):
        i = 0
        for element in self.header.values():
            self.message[i:i+element.data_size] = element.serialize()
            i += element.data_size
        self.message[i] = 0

    def get_element(self, element):
        if element in self.header:
            return self.header[element].data
        else:
            print("Invalid elemet")

    def get_element_raw(self, size, index):
        return int.from_bytes(self.message[index:index+size], byteorder = ENDIAN, signed=False)

    def set_element(self, element, value):
        if element in self.header:
            self.header[element].data = value
        else:
            print("Invalid elemet")

    

if __name__ == '__main__':
    #  a = Data_Message()
    #  print(a.message)
    #  print(a.get_header_element("PROTOCOL"))
    #
    #  b = Data_Message(a.message)
    #  print(b.message)
    #  b.serialize_data(5, 4, 'i')
    #  print(b.message)
    #  b.serialize_data("Hello", len("Hello") + 1, 's')
    #  print(b.message)
    #  b.serialize_data('g', 10, 's')
    #  print(b.message)
    #  b.serialize_header()
    #  print(b.message)
    #  print(len(b.message))
    #  b.serialize_data("Hello", len("Hello") + 1, 's')
    #  b.serialize_data("Hello", len("Hello") + 1, 's')
    #  b.serialize_header()
    #  print(len(b.message))
    #  print(b.header["BYTE_LENGTH"].data)
    a = Ping_Message()
    print(a.message)
    a.serialize()
    print(a.message)
    a.set_element("TYPE", 4)
    print(a.message)
    a.serialize()
    print(a.message)
    b = Ping_Message(a.message)
    print(b.message)

