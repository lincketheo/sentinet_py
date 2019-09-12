
from control.ControlClient import ControlClient
import messages.proto
import time
from control import GLPDC


def callback_server(val):
    print("Server sending: " + val)
    return "Server response " + val

def callback_client(val):
    print("Client recieved: " + val)

if __name__ == '__main__':

    a = ControlClient(True, (False, ""))

    a.serve(address = "tcp://*:5555", callback = callback_server, start_on_creation=True)

    val = str(a.request_concurrent(address = "tcp://localhost:5555", message = "Hi there"))

    print("Recieved: " + val)
    time.sleep(4)

    a.quit()
"""
from control import *

KermitAlphabet = {"x" : math.inf,
                  "y" : math.inf,
                  "theta" : math.inf,
                  "auger" : True,
                  "mining" : False,
                  "dumping" : False,
                  "bucket_full" : False,
                  "moving" : False,
                  "system" : False,
                  "time" : 600,
                  "kill_flag" : False,
                  "start_flag" : False}


init = InitState(0, "Initializing")
drive = DriveState(1, "Driving")
mine = MineState(2, "Mining")
dump = DumpState(3, "DumpState")
kill = KillState(4, "Killed")

init.associate_transition_function("lambda1", drive)
init.associate_transition_function("kill", kill)

drive.associate_transition_function("lambda2", mine)
drive.associate_transition_function("lambda3", dump)
drive.associate_transition_function("kill", kill)

mine.associate_transition_function("lambda4", drive)
mine.associate_transition_function("kill", kill)

dump.associate_transition_function("lambda5", drive)
dump.associate_transition_function("kill", kill)

states = [drive, init, kill, mine, dump]

ksm = StateMachine(init, KermitAlphabet, states, "A Simple State Machine")

    
if __name__ == '__main__':
    # Start initializing
    print(ksm.current_state.get_desc())
    # Transition, nothing happens because our alphabet is stagnant
    ksm.transition()
    print(ksm.current_state.get_desc())

    # Change that alphabet yo
    ksm.alphabet.assign_var("start_flag", True)
    # Transition, we should be driving
    ksm.transition()
    print(ksm.current_state.get_desc())

    ksm.alphabet.assign_var("mining", True)
    ksm.transition()
    print(ksm.current_state.get_desc())

    # Now lets throw an invalid state - nothing changes!
    ksm.alphabet.assign_var("dumping", True)
    ksm.transition()
    print(ksm.current_state.get_desc())

    # This should change to driving, but dumping is also true!
    ksm.alphabet.assign_var("driving", True)
    ksm.transition()
    print(ksm.current_state.get_desc())
"""
