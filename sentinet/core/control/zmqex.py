from control.ControlClient import ControlClient
import messages.proto
import time
from control import GLPDC, StateMachine, State, Alphabet

starting_state = State(0, "Starting")
driving_state = 


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
