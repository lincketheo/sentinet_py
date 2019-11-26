#!/usr/bin/env python3 
import sys
from struct import pack
from sentinet.curmt import KermitControlModule, Driver
import time
import random

if __name__ == '__main__':
    a = Driver()
    a.start()

    a.forward(30.0)
    a.turn(0.0)
    print("Moving forwards")
    time.sleep(7)

    a.forward(0.0)
    a.turn(30.0)
    print("Turning")
    time.sleep(3)

    a.forward(30.0)
    a.turn(6.0)
    print("Moving forward and turning")
    time.sleep(3)

    # Stop does the same thing
    a.forward(0.0)
    a.turn(0.0)
    print("Stopping")

    a.end()
