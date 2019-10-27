#!/usr/bin/env python3 

import sys
from struct import pack
from sentinet.core.messages.Message import *
from sentinet.curmt import KermitControlModule
import time
import random


def example_get_data():
    a = random.random()
    b = random.random()
    print(a, b)
    return a, b

if __name__ == '__main__':
    a = KermitControlModule()

    a.set_cmd_vel_get_data(example_get_data)

    a.start_kermit()

    time.sleep(50)

    a.quit_kermit()
