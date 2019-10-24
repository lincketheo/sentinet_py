from sentinet.core.control import ControlClient
from sentinet.core.messages.Message import Data_Message, Ping_Message

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

        self.data_buffer = Data_Message()
        self.ping_buffer = Ping_Message()
        # TODO NEED TO IMPLIMENT MESSAGE BUFFER HERE

    def get_data(self) -> bytes:
        print("Sending data")
        return "temporary".encode('utf-8')

    def start_kermit(self):
        self.control = ControlClient(False, (False, ""))
        self.control.publish(PUB_ADDR, cmd_vel, self.get_data, 1, True)
        
    def loop_kermit(self):
        self.__update_state__()

    def quit_kermit(self):
        self.control.quit()

    def set_linear(self, value: float):
        self.linear = value

    def set_angular(self, value: float):
        self.angular = value

    def zero_motors(self):
        self.set_angular(0.0)
        self.set_linear(0.0)

    def trigger_dumping(self):
        self.dumping = not self.dumping

    def trigger_mining(self):
        self.mining = not self.mining

    def set_dumping(self, value: bool):
        self.dumping = value

    def set_mining(self, value: bool):
        self.mining = value
