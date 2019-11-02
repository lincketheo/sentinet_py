from sentinet.core.control import ControlClient, pub_params, sub_params
from sentinet.core.messages.Message import Data_Message, Ping_Message
from sentinet.core.messages.MessageKeys import *
from struct import pack

CMD_VEL = "tcp://localhost:5570"
DATA_ADDR = "tcp://localhost:5556"
COMMAND_ADDR = "tcp://localhost:5572"
REAL_TIME_ADDR = "tcp://localhost:5573"

class KermitControlModule:

    def __init__(self, requesting = False, publishing = False):

        # Control Client
        self.control = None
    
        # Real time requsting channel 
        self.requesting = requesting 
        # Real time publish channel
        self.publishing = False

        # Messages
        if self.requesting:
            self.command_data = Ping_Message()
        else:
            self.command_data = None

        # Params
        self.cmd_vel = None
        self.cmd_vel_callback = None

        self.data = None
        self.data_msg = None
        self.data_callback = None

        self.command = None
    #  Checks to see if the requester is set to pass otherwise prints an error message.
    def __request(self, message):
        if not self.requesting:
            print("Error, you didn't pass True to requesting on construction")
            return "" 
        return self.control.request_concurrent(COMMAND_ADDR, message)

    def request(self, head: int, code: int, excess: int):
        self.command_data.set_type(head)
        self.command_data.set_code(code)
        self.command_data.set_excess(excess)
        return self.__request(bytes(self.command_data.message))

    def start_kermit(self):
        # Start control module
        self.control = ControlClient(self.requesting, (False, ""))
        # Start data subscriber if implimented
        self.control.spin_subscriber(self.data) if self.data is not None else print("Data not implimented")
        # Start publisher module if implimented
        self.control.spin_publisher(self.cmd_vel) if self.cmd_vel is not None else print("Cmd vel not implimented")
        
    # Stop kermit
    def quit_kermit(self):
        self.control.quit()

    def __cmd_vel_get_data(self):
        a, l = self.cmd_vel_callback()
        self.cmd_vel_data.set_data(a, 4, FLOAT, 0)
        self.cmd_vel_data.set_data(l, 4, FLOAT, 1)
        return bytes(self.cmd_vel_data.message)

    def set_cmd_vel_get_data(self, func):
        # Initialize message space
        self.cmd_vel_data = Data_Message()
        self.cmd_vel_data.push_data(0.0, 4, FLOAT)
        self.cmd_vel_data.push_data(0.0, 4, FLOAT)
        self.cmd_vel_data.to_wire()

        # Nothing to do with CC
        self.cmd_vel_callback = func

        # Create a new pub params
        self.cmd_vel = pub_params()
        # The address to publish to 
        self.cmd_vel.address = CMD_VEL
        # The callback to get data
        self.cmd_vel.get_data = self.__cmd_vel_get_data
        # The topic
        self.cmd_vel.topic = "cmd_vel"
        # The period to publish on
        self.cmd_vel.period = 0.01
        # The start on creation
        self.cmd_vel.start_on_creation = True 

    # func gets two floats and returns void
    def data_callback_(self, incomming_message):
        self.data_msg.parse_from_similar_message(incomming_message)
        a = struct.unpack('f', self.data_msg.get_data(0))[0]
        b = struct.unpack('f', self.data_msg.get_data(1))[0]
        print(self.data_msg.message)
        self.data_callback(float(a), float(b))

    def set_data_callback(self, func):
        self.data_msg = Data_Message()
        self.data_msg.push_data(0.0, 4, FLOAT)
        self.data_msg.push_data(0.0, 4, FLOAT)

        # Nothing to do with CC
        self.data_callback = func

        # Create a new sub params
        self.data = sub_params()
        # Connect to address
        self.data.address = DATA_ADDR
        # Attach callback, takes in a string, needs a wrapper function
        self.data.callback = self.data_callback_
        # Attach the data topic
        self.data.topic = "data"
        # Start on creation
        self.data.start_on_creation = True
