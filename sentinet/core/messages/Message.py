import sys
from struct import pack
from sentinet.core.messages.MessageKeys import *


class Data_Message:

    def __init__(self, incomming_message=0): 

        # Recieve the default header for backwards compatability sake
        self.header = default_header

        # Get the size of the header (TODO - this is still to explicit)
        self.header_size = self.header["FIELDS"].index + self.header["FIELDS"].data_size

        # Recieving a message
        if incomming_message is not 0:
            self.message = incomming_message

            # update values by reading elements from within the buffered message
            for i in self.header.values():
                i.data = self.get_header_element_raw(i.data_size, i.index) 

        # Not recieving a message, so we need to make a new message
        else:
            self.message = bytearray(self.header_size) 
            self.message.append(0)
            if not self.serialize_header():
                print("Exiting")
                sys.exit(1)


    # Emplaces all header elements in self.header
    # into the message buffer
    def serialize_header(self):
        i = 0 
        # For every data element, serialize it into the header
        for element in self.header.values():
            # Check for indexes and make sure they match
            if i != element.index:
                print("Error - invalid header index")
                return False

            # Otherwise, serialize the data element
            self.message[i:i+element.data_size] = element.serialize()

            # Incriment index by data size so there is no structure padding
            i += element.data_size

        # Add a null terminator
        self.message[i] = 0
        return True
         
    # Returns element that is in the header
    def get_header_element(self, element):
        # This checks for the map not the message
        # bytearray
        if element in self.header:
            return self.header[element].data
        else:
            print("Invalid elemet")

    # Returns element that is in the message header
    def get_header_element_raw(self, size, index):
        # Utilizing the fact that all our header elements can be expressed as 
        # unsigned ints of arbitrary sizes - a little too explicit, may want to change
        return int.from_bytes(self.message[index:index+size], byteorder = ENDIAN, signed=False)

    # Serialize a data element (not in the header that is)
    def serialize_data(self, data, bytes_size, type_code):
        # Check data type, otherwise, we
        # simply represent it as an int
        # TODO add more cases here - maybe a switch table
        if(type(data) == str):

            # The byte size should always be 1 more than the size of the string 
            # To account for c style null terminator
            if(bytes_size != len(data) + 1):
                print("ERROR")
                return 1

            # Add a string null terminator for C 
            # and turn it into bytes
            data = data.encode('utf-8') + b"\x00"

        # simply convert the data to bytes
        # TODO im pretty sure this is fine for floats, but I'd have to check
        elif(type(data) == int):
            # Convert data to bytes
            data = data.to_bytes(bytes_size, byteorder = ENDIAN, signed = False)
        elif(type(data) == float):
            # Convert data to bytes
            data = pack('f', data)

        # Append data

        # First, the message size (1 byte)
        self.message.append(bytes_size)

        # Second the message type code in utf (1 byte)
        self.message.append(ord(type_code))

        # Third, the data
        self.message.extend(data)

        # Fourth the null terminator
        self.message.append(0)

        # Update header fields and byte length
        self.header["FIELDS"].data += 1
        self.header["BYTE_LENGTH"].data += 3 + bytes_size

    def set_data(self, data: bytes, byte_size: int, type_code, byte_index):
        if self.header["BYTE_LENGTH"].data < byte_index + byte_size + 3:
            print("Index to high, pushing to the top of stack")
            self.serialize_data(data, byte_size, type_code)
            return True
        # Itterate through the message
        print(self.message)
        self.message[byte_index] = byte_size
        self.message[byte_index + 1] = ord(type_code)
        self.message[byte_index + 2:byte_index + byte_size + 2] = data
        self.message[byte_index + byte_size + 2] = 0
         
# A ping message is a lot easier to work with
# Because it's one big header
class Ping_Message:

    def __init__(self, incomming_message = 0):

        # Set it as default to have a standard even if 
        # we simply recieve the message
        self.header = default_ping
        self.header_size = default_ping["EXCESS"].index + default_ping["EXCESS"].data_size + 1

        # If we have no incomming message, create a new packet
        # This could be done in a ternerary, but there may be more we 
        # want to do so I'm leaving it as if else
        if incomming_message == 0:
            self.message = bytearray(self.header_size)

        else: 
            # Check the incomming message - see below TODO
            if self.header_size != len(incomming_message):
                print("Error incomming message size does not match a standard ping message")

            # Copy the message over
            self.message = incomming_message

            # Then itterate through to update the dict values
            for i in self.header.values():
                i.data = self.get_header_element_raw(i.data_size, i.index) 

            # TODO - put all incomming message checks in their
            # own method called validate incomming message
            # Right now, I'm just serializing and expecting
            # nothing to change
            if not self.serialize():
                print("Exiting")
                sys.exit(1)

            if self.message != incomming_message:
                print("Error parsing incomming message")

    # Serialize the data within our message
    def serialize(self):
        i = 0

        # Itterate through all items in the dict and update byte array
        for element in self.header.values():

            # Check for index matching
            if i != element.index:
                print("Error serializing ping, indexes are off")
                return False

            # Otherwise, copy data into the buffer and increment the index
            self.message[i:i+element.data_size] = element.serialize()
            i += element.data_size

        # A null terminator
        self.message[i] = 0
        return True

    # Gets element at a specified key
    def get_element(self, element):
        if element in self.header:
            return self.header[element].data
        else:
            print("Invalid elemet")

    # Gets element at specified index and size in byte form
    def get_element_raw(self, size, index):
        return int.from_bytes(self.message[index:index+size], byteorder = ENDIAN, signed=False)

    # Sets data element in dict (not message)
    def set_element(self, element, value):
        if element in self.header:
            self.header[element].data = value
        else:
            print("Invalid elemet")

    

if __name__ == '__main__':

    # Example:
    dm1 = Data_Message()

    # Serialize some data
    dm1.serialize_data("Hello", 6, STRING)
    dm1.serialize_data(5.9999123123, 4, FLOAT)

    # Shouldn't serialize because the length does not
    # account for null terminator
    print("Expecting ERROR: ")
    dm1.serialize_data("invalid", len("invalid"), STRING)

    # Should serialize
    dm1.serialize_data("valid", len("valid") + 1, STRING)
    
    # Update the header
    dm1.serialize_header()

    # Print some data
    print(dm1.message)
    print("Header byte length: " + str(dm1.header["BYTE_LENGTH"].data))
    print("Actual byte length: " + str(len(dm1.message)))

    # Note that get_header_element_raw should never be used outside,
    # I'm just using it to demonstrate
    print("Byte length in mes: " + str(dm1.get_header_element_raw(4, 6)))

    dm2 = Data_Message(dm1.message)
    dm2.serialize_header()

    # Print some data

    print(dm2.message)
    print("Header byte length: " + str(dm2.header["BYTE_LENGTH"].data))
    print("Actual byte length: " + str(len(dm2.message)))

    # Note that get_header_element_raw should never be used outside,
    # I'm just using it to demonstrate
    print("Byte length in mes: " + str(dm2.get_header_element_raw(4, 6)))


    pm1 = Ping_Message()
    pm1.set_element("TYPE", 0x2)
    pm1.set_element("CHECKSUM", 0x3)
    pm1.set_element("CODE", 1)
    pm1.set_element("EXCESS", 123123123)
    pm1.serialize()
    print(pm1.message)
    print("123123123 = " + str(pm1.get_element_raw(8, 7)))
    print("3 = " + str(pm1.get_element_raw(2, 5)))
    print("1 = " + str(pm1.get_element_raw(2, 3)))
    print("2 = " + str(pm1.get_element_raw(1, 2)))
