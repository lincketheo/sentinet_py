import sys
from struct import pack
from sentinet.curmt import KermitControlModule
import time
import random


STATE_REQUEST = 8
STOP_EVERYTHING = 1
DUMP = 1 << 1
MINE = 1 << 2
MOVE_TO_DUMP = 1 << 3
MOVE_TO_MINE = 1 << 4
INIT_STATE = 1 << 5
CLEAN_EXIT = 1 << 6
MICRO_MOVEMENT = 1 << 7


ENTERING_STATE = 1 << 0
EXIT_STATE = 1 << 4


class ControlExtension:

    def __init__(self):
        self.lin = 0.0
        self.ang = 0.0

        self.cc = KermitControlModule(requesting = True)
        self.cc.set_cmd_vel_get_data(self.cmd_vel_get_data)
        self.cc.set_data_callback(self.incomming_data_callback)

    # Start the endpoint
    def start(self):
        self.cc.start_kermit()

    # Stop the comms endpoint
    def stop(self):
        self.cc.quit_kermit()

    # Sets heading an desired linear magnitude
    def set_desired_pos(self, degree: float, magnitude: float):
        self.lin = magnitude
        self.ang = degree

    # enter A desired state (see above for state codes)
    def enter_state(self, state: int):
        self.request(STATE_REQUEST, state, ENTERING_STATE)

    # exit a desired state (see above for state codes)
    def exit_state(self, state: int):
        self.request(STATE_REQUEST, state, EXIT_STATE)

    # Start driving in general, will remove this
    def start_driving(self):
        self.request(STATE_REQUEST, MOVE_TO_MINE, 1 << 1)

    # Stop driving in general, will remove this
    def stop_driving(self):
        self.request(STATE_REQUEST, STOP_EVERYTHING, 0)

    # reequest from a server
    def request(self, head:int, code = 0, excess = 0):
        return self.cc.request(head, code, excess)

    # The callback for getting data
    def cmd_vel_get_data(self):
        return self.lin, self.ang


if __name__ == '__main__':
    a = ControlExtension()

    a.start()

    # Do work, or loop or whatever
    # This updates the robot and sends 
    # lin and ang
    #  for i in range(1000):
    #      a.lin = random.random() * 30
    #      a.ang = random.random() * 30
    #      time.sleep(5)

    a.lin = 0.0
    a.ang = 0.0
    time.sleep(5)
    # Note that publisher is still sending
    # while this call happens, sometimes, 
    # this call blocks, but publisher is still
    # sending data, don't worry
    #  ret = a.request(5, 6, 7)
    #  print(f"Responded with {ret}")
    #  ret = a.request(5, 9, 0)
    #  print(f"Responded with {ret}")

    a.stop()
