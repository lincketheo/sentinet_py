#!/usr/bin/env python3 
import sys
from struct import pack
from sentinet.curmt import KermitControlModule
import time
import random

"""
This is an example implimentation of Kermit Control Module

Kermit Control Module is an extension of the control client
The main thing to remember is to set callbacks 
"""
class ExampleControlExtension:

    def __init__(self):
        self.lin = 5.6
        self.ang = 5.4

        # By saying requesting = True, its telling
        # kcm that I want to be able to request inline
        # We'll set this to true for all our CC's
        self.cc = KermitControlModule(requesting = True)

        # SETTERS
        """
        Setters start kcm on a specified channel,
        if you don't set any of these, nothing will
        happen. On the other hand, the ones you set 
        open channels, so if you only want to listen / 
        publish on one channel, then only open that one 
        channel. 
        """

        # set_cmd_vel_get_data expects a function
        # that returns two floats
        self.cc.set_cmd_vel_get_data(self.cmd_vel_get_data)

        # set_data_callback expects (FOR NOW) a function
        # that takes in two floats and returns void
        self.cc.set_data_callback(self.incomming_data_callback)

        # Soon, we'll have a real time map channel as well


    # START AND STOPPERS
    """
    At the very least, you need to start and stop kermit you could do this
    in __init__, but then you'd need to quit kermit somewhere. You
    may need to do nothing here, but have these any way just so that you
    can stop and start kermit.
    """
    # Just start control client and anything else you need
    def start(self):
        # Add stuff here for init sequence
        self.cc.start_kermit()

    # Just stop control client and anything else you need
    def stop(self):
        # Add stuff here for init sequence
        self.cc.quit_kermit()

    """
    Technically not necessary, but this requests using a ping
    message (with type = head, code = code, and excess = excess)

    Other option would just be to use cc as a handle

    The parameters are the head code and excess for a ping message
    as talked about in the meeting
    """
    def request(self, head:int, code = 0, excess = 0):
        return self.cc.request(head, code, excess)

    # CALLBACKS
    # An example incomming data callback
    def incomming_data_callback(self, lin: float, ang: float):
        print(f"I recieved: {lin}, {ang}")
        return

    # Get data callback example
    # Expects two floats for now
    def cmd_vel_get_data(self):
        return self.lin, self.ang


if __name__ == '__main__':
    a = ExampleControlExtension()

    a.start()

    # Do work, or loop or whatever
    # This updates the robot and sends 
    # lin and ang
    for i in range(5):
        a.lin = random.random()
        a.ang = random.random()
        time.sleep(1)

    # Note that publisher is still sending
    # while this call happens, sometimes, 
    # this call blocks, but publisher is still
    # sending data, don't worry
    ret = a.request(5, 6, 7)
    print(f"Responded with {ret}")
    ret = a.request(5, 9, 0)
    print(f"Responded with {ret}")
    
    for i in range(5):
        a.lin = random.random()
        a.ang = random.random()
        time.sleep(1)

    a.stop()
