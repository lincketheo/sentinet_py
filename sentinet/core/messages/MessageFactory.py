import sys
from struct import *


class MessageFactory:

	type_sizes = {'c':1, '?':1, 'f':4, 'd':8, 'I':4}
	# char, bool, float, double, int

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


	def __generate_header(self):
		return self.endianness + self.mtype + self.flags + self.protocol + self.bytesize + self.pad + self.pad

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


	def deserialize_data(self, string):
		return 0


if __name__ == '__main__':
	mf = MessageFactory('float',2)
	print(mf.serialize_data([0.2423,0.4245]))
	
	