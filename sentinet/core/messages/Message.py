import sys
from struct import pack
from sentinet.core.messages.MessageKeys import *
import copy

##
# @brief A Data Message that holds Data
class Data_Message:

    ##
    # @brief Default constructor
    #
    # @return A new Data message
    def __init__(self): 
        self.header = copy.deepcopy(default_header)
        # + 1 for null terminator case
        self.header_size = self.header["FIELDS"].index + self.header["FIELDS"].data_size + 1
        self.message = bytearray(self.header_size) 
        # Serialize header, if an error occurs here
        # it's not a runtime issue, its a versioning one
        if not self.__serialize_header():
            print("Exiting")
            sys.exit(1)
        # For faster retrivals of data, 
        # store the indexes of each element in indices
        self.indices = []

    ##
    # @brief Serializes data in self.header into header buffer
    #
    # @return Status
    def __serialize_header(self):
        i = 0 
        # For every data element, serialize it into the header
        for element in self.header.values():
            if i != element.index:
                print("Error - invalid header index")
                return False
            self.message[i:i+element.data_size] = element.serialize()
            i += element.data_size
        # Add a null terminator
        self.message[i] = 0
        return True

    ##
    # @brief Gets data from the header buffer
    #
    # @return Status
    def serialize_from_header(self):
        i = 0 
        for element in self.header.values():
            if i != element.index:
                print("Error - invalid header index")
                return False
            element.data = int.from_bytes(self.message[i:i+element.data_size], byteorder = ENDIAN, signed = element.signed)
            i += element.data_size
        return True
     
    ##
    # @brief Retrieves a value from the header buffer
    #
    # @param size The size of the value
    # @param index The index to get
    #
    # @return The value at that index
    def get_header_element_raw(self, size, index):
        if index + size > self.header_size:
            print("Error, invalid range")
            return None
        return int.from_bytes(self.message[index:index+size], byteorder = ENDIAN, signed=False)

    ##
    # @brief Pushes a new data element to the end of data message (see set_data for alternative)
    #
    # @param data The "raw" data to serializable by the codes defined in MessageKeys.py
    # @param bytes_size The size of the data - does some checks to make sure this is right
    # @param type_code The type code, see MessageKeys.py
    #
    # @return Status of serialization
    def push_data(self, data, bytes_size, type_code):
        # Do the preliminary checks on the data
        if type_code in serialize_funcs:
            if serialize_funcs[type_code][0](data, bytes_size):
               data = serialize_funcs[type_code][1](data)
            else:
                print("Invalid data type for declared type code")
                return False
        else:
            print("Invalid type code")
            return False
        
        # Serialize the data to the end
        self.indices.append(self.header["BYTE_LENGTH"].data) # Change this
        self.message.append(bytes_size)
        self.message.append(ord(type_code))
        self.message.extend(data)
        self.message.append(0)
        self.header["FIELDS"].data += 1
        self.header["BYTE_LENGTH"].data += 3 + bytes_size

    ##
    # @brief Sets data within a message, idealy for when you know the attributes of the message
    #
    # @note This function sets a new data element and updates indices
    #
    # @param data The data to serialize
    # @param bytes_size The size of the element
    # @param type_code The type code
    # @param index The index to serialize into
    #
    # @return Status of serialization
    def set_data(self, data, bytes_size, type_code, index):
        if index >= self.header["FIELDS"].data or bytes_size > 255:
            print("Invalid index")
            return False
        if type_code in serialize_funcs:
            if serialize_funcs[type_code][0](data, bytes_size):
                data = serialize_funcs[type_code][1](data)
            else:
                print("Invalid data type for declared type code")
                return False
        else:
            print("Invalid type code")
            return False

        # Index of data
        i = self.indices[index]
        # Size of data at index
        size = int.from_bytes(self.message[i:i+1], byteorder = ENDIAN)

        # Optimal operations for three cases
        if size == bytes_size:
            self.message[i] = bytes_size
            self.message[i + 1] = ord(type_code)
            self.message[i+2 : i+2+bytes_size] = data

        elif size > bytes_size:
            self.message[i] = bytes_size
            self.message[i + 1] = ord(type_code)
            self.message[i+2 : i+2+bytes_size] = data
            del self.message[i+2+bytes_size : i+2+size]
            diff = size - bytes_size
            for i in range(len(self.indices) - index - 1):
                self.indices[i + index + 1] -= diff

        elif size < bytes_size:
            diff = bytes_size - size
            self.message[i] = bytes_size
            self.message[i + 1] = ord(type_code)
            # Figured out that python automatically
            # allocates extra space for data if data
            # size is larger
            self.message[i + 2:i + 2 + size] = data
            for i in range(len(self.indices) - index - 1):
                self.indices[i + index + 1] += diff

        
        self.header["BYTE_LENGTH"].data += (bytes_size - size)

    ##
    # @brief Gets byte data at a specified index
    #
    # @param index The index to retrieve
    #
    # @return The data
    def get_data(self, index):
        if index >= self.header["FIELDS"].data:
            print("Invalid index")
            return None
        index = self.indices[index]
        size = self.message[index]
        return self.message[index+2 : index+2+size]

    ##
    # @brief Parse from a message and update self attributes
    #
    # @note This shouldn't be used real time. See next method
    #
    # @param message The incomming message
    #
    # @return Status of parse
    def parse_from_message(self, message):
        self.message = message
        self.indices = []
        
        i = 13
        if self.message[0] > 1:
            return 1
        if self.message[12] != 0:
            return 2
        bytes_size = int.from_bytes(self.message[self.header["BYTE_LENGTH"].index: \
                     self.header["BYTE_LENGTH"].index + self.header["BYTE_LENGTH"].data_size], byteorder = ENDIAN)
        if bytes_size != len(self.message):
            return 3
        while i < bytes_size:
            if self.message[i] == 0:
                return 4
            size = self.message[i] 
            if 3 + size + i > bytes_size:
                return 5
            self.indices.append(i)
            i += size + 2
            if self.message[i] != 0:
                return 6
            i += 1
        self.serialize_from_header()
        return True
                        
    ##
    # @brief Parse a message that has the same structure as this one
    #
    # @param message The incomming message
    #
    # @return The status of parse
    def parse_from_similar_message(self, message):
        if self.header["BYTE_LENGTH"].data != len(message):
            return False
        self.message = message
        for i in range(len(self.indices)):
            if self.message[self.indices[i] - 1] != 0:
                return False
        return True

    def to_wire(self):
        self.__serialize_header()
        

# A ping message is a lot easier to work with
# Because it's one big header
class Ping_Message:
    def __init__(self):
        self.header_size = 15
        self.message = bytearray(self.header_size)

    def set_protocol(self, value):
        self.message[0] = value

    def set_opcode(self, value):
        self.message[1] = value

    def set_type(self, value):
        self.message[2] = value

    def set_code(self, value):
        self.message[3:5] = value.to_bytes(2, byteorder = ENDIAN, signed = False)

    def set_checksum(self, value):
        self.message[5:7] = value.to_bytes(2, byteorder = ENDIAN, signed = False)

    def set_excess(self, value):
        self.message[7:15] = value.to_bytes(8, byteorder = ENDIAN, signed = False)
