import sys
from struct import pack
from sentinet.core.messages.Message import *
from sentinet.core.control.ControlClient import *
import time

temp = Data_Message()
temp.push_data(0.0, 4, FLOAT)
temp.push_data(0.0, 4, FLOAT)


def callback(value):
    temp.parse_from_similar_message(value)
    a = struct.unpack('!f', bytes(temp.get_data(0)))[0]
    b = struct.unpack('!f', bytes(temp.get_data(1)))[0]
    print(f'Recieved {a} {b}')
    print(a + b)

if __name__ == '__main__':
    a = ControlClient()

    sub = sub_params()
    sub.address = "tcp://localhost:5571"
    sub.callback = callback
    sub.start_on_creation = True
    sub.topic = "cmd_vel"

    a.spin_subscriber(sub)

    a.start()

    time.sleep(6)

    a.quit()

