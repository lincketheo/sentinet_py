from sentinet.core.messages.Message import Data_Message, Ping_Message
from sentinet.core.messages.MessageKeys import *




if __name__ == '__main__':
    
    """
    There are two philosopies to making messages:

        1. Create the message structure beforehand, then 
        keep the same structure throughout

        2. Make the message structure dynamic

    I'll show both, number 1 first because it's the better (faster) way
    """

    # Create a new message
    data_float_pair = Data_Message()

    """
    For this example, consider a message that consists of a float,
    an int, and a string
    """

    # Takes in parameters:
    # 1. The data
    # 2. The size (a float is 4 bytes)
    # 3. The type code, see sentinet.core.messages.MessageKeys for a list of them
    data_float_pair.push_data(0.0, 4, FLOAT)
    data_float_pair.push_data(0, 4, UINT32)
    # The plus one on the string is for the null terminator
    # Python doesn't add null terminators, the program will yell at
    # you if you don't add one to the length
    data_float_pair.push_data("Hello World", len("Hello World") + 1, STRING) 

    # to_wire does some stuff to the message header, call it everytime
    # the message size changes or the number of elements changes
    data_float_pair.to_wire()
   

    """
    Now, we can mind our business and access the raw data like this:
    Note that these return byte arrays
    """
    # The entire message
    print(data_float_pair.message)
    # Indexed messages
    print(data_float_pair.get_data(0))
    print(data_float_pair.get_data(1))
    print(data_float_pair.get_data(2))

    """
    And we can set the data in the message like so:
    """
    # The zero at the end is the index (the float was at index 0)
    data_float_pair.set_data(1.5123, 4, FLOAT, 0)
    data_float_pair.set_data(9, 4, UINT32, 1)
    data_float_pair.set_data("Same size s", len("Same size s") + 1, STRING, 2)

    # No need to call to_wire because the structure didn't change
    print(data_float_pair.message)


    """
    Heres the second way to use messages: dynamically

    I'm using the same example, but all the above functions can be used the same way

    We can push more data:
    """
    data_float_pair.push_data("Hi there, buddy", len("Hi there, buddy") + 1, STRING)
    data_float_pair.push_data(1, 2, UINT16)
    print(data_float_pair.message)
    print(data_float_pair.get_data(0))
    print(data_float_pair.get_data(1))
    print(data_float_pair.get_data(2))
    print(data_float_pair.get_data(3))

    """
    We can also set the data of an element in the middle. For example, below,
    I'm setting the data at index 0 (where the float used to be). This changes
    the structure of the message, but you can still access data the same way
    (i.e by index)
    """
    data_float_pair.set_data(ord('h'), 1, INT8, 0)
    data_float_pair.to_wire()

    print(data_float_pair.message)
    print(data_float_pair.get_data(0))
    print(data_float_pair.get_data(1))
    print(data_float_pair.get_data(2))
    print(data_float_pair.get_data(3))
