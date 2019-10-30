import sys
from struct import *

# Generates the behaviors and features of the a message storing the header, note has special configuration for serialized data.
class MessageFactory:

	type_sizes = {'c':1, '?':1, 'f':4, 'd':8, 'I':4}
	# char, bool, float, double, int

	##
    # @brief Stores by default the features of a member storing default data storage type to an eight bit unicode.
    #
    # @note I do not fully understand what I am doing, will study more later.
    #
    # @param data_type The data type used to store the information of the message.
	# @param num The index of message.
    #
    # @return The initialized object of the message factory
	def __init__(self, data_type, num):
		self.endianness = pack('c',bytes(sys.byteorder[0],'utf-8'))
		self.typecode = data_type[0]
		self.pad = pack('x')
		self.mtype = pack('c',bytes('0','utf-8'))
		self.flags = pack('c',bytes('0','utf-8'))
		self.protocol = pack('c',bytes('1','utf-8'))
		self.num = num
		self.data_size = self.type_sizes[self.typecode]
		self.bytesize = pack('I',13 + num * (3+self.data_size))


    ##
    # @brief Constructs a header for an object of class MessageFactory
    #
    # @note I have no idea what I am doing.
    #
    #
    # @return A default header that stores all basic information of objects of the class.
	def __generate_header(self):
		return self.endianness + self.mtype + self.flags + self.protocol + self.bytesize + self.pad + self.pad

    ##
    # @brief Searlizes the data of a MessageFacotyr for sending.
    #
    # @note I have no idea what I am doing.
    #
    # @param data The content of MessageFactory self that is being standardized.
    #
    # @return The message of the function being standardized.
	def serialize_data(self, data):
		assert len(data) is self.num, "Data length doesn't match factory."
		message = self.__generate_header()
		for dat in data:
			size_of_field = pack('I',self.data_size)
			type_of_field = pack('c',bytes(self.typecode,'utf-8'))
			field = pack(self.typecode,dat)
			message += size_of_field + type_of_field + field + self.pad
		message = message + self.pad
		return message

	##
    # @brief Extracts content from a message for use.
    #
    # @note I have no idea what I am doing.
    #
    # @param string Possibly the defining features of the function being accessed.
    #
    # @return supposed to convert a message for processing and use.
	def deserialize_data(self, string):
		return 0


if __name__ == '__main__':
	mf = MessageFactory('float',2)
	print(mf.serialize_data([0.2423,0.4245]))
	
	