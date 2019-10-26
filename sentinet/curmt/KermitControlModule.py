from sentinet.core.control import ControlClient
from sentinet.core.messages.Message import Data_Message, Ping_Message
from sentinet.core.messages.MessageKeys import *
from struct import pack

cmd_vel = "cmd_vel"



PUB_ADDR = "tcp://localhost:5555"
SUB_ADDR = "tcp://localhost:5556"
SERVE_ADDR = "tcp://localhost:5557"
REQ_ADDR = "tcp://localhost:5558"


class KermitControlModule:

    def __init__(self):
        self.control = None
        self.pub_addr = PUB_ADDR
        self.linear = 0.0
        self.angular = 0.0
        self.mining = False
        self.dumping = False

        self.cmd_vel = None
        self.data = None
        self.data_callback = None
        self.command = None

    def request(self, data):
                
        return

    def start_kermit(self):
        self.control = ControlClient(False, (False, ""))
        if self.data is not None:
            self.control.subscribe(self.data)
        else
            print("Data not implimented")
        
    def quit_kermit(self):
        self.control.quit()
        return 

    def set_cmd_vel_get_data(self, func):
        self.cmd_vel = pub_params()
        self.cmd_vel.address = "tcp://localhost5556"
        self.cmd_vel.get_data = func
        self.cmd_vel.topic = "cmd_vel"
        self.cmd_vel.period = 1
        self.cmd_vel.start_on_creation = True 
        return

    # func gets two floats and returns void
    def data_callback(self):
        if self.data_callback is not None:
            a = 5.6
            b = 7.1
            self.data_callback(a, b)

    def __set_data_callback(self, func):
        self.data_callback = func
        self.data = sub_params()
        self.data.callback = self.data_callback
        self.data.topic = "data"
        self.data.start_on_creation = True
        return


