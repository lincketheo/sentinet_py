import zmq
import threading
import time
import sys
from sentinet.core.messages.Message import Data_Message
from sentinet.core.messages.MessageKeys import FLOAT
import struct
import random

server_port = "5572"
publisher_port = "5556"

def server_thread():
    context = zmq.Context()
    socket = context.socket(zmq.REP)
    socket.bind("tcp://*:%s" % server_port)

    while True:
        #  Wait for next request from client
        message = socket.recv()
        print ("Received request: ", message)
        time.sleep (1)
        socket.send(b"Hello there")

message = Data_Message()
message.push_data(0.0, 4, FLOAT)
message.push_data(0.0, 4, FLOAT)
message.to_wire()

def publisher_thread():
    context = zmq.Context()
    socket = context.socket(zmq.PUB)
    socket.bind("tcp://*:5556")

    while True:
        message.set_data(random.random(), 4, FLOAT, 0)
        message.set_data(random.random(), 4, FLOAT, 1)
        print("Sending: ", bytes(message.message))
        time.sleep(1)
        socket.send_multipart([b"data", bytes(message.message)])

if __name__ == '__main__':
    t1 = threading.Thread(target = server_thread)
    t2 = threading.Thread(target = publisher_thread)

    t1.start()
    t2.start()

    t1.join()
    t2.join()

