from ControlClient import ControlClient
import time


def get_data():
    print("Sending data")
    return "hi there"

if __name__ == '__main__':
    a = ControlClient(False, (False, ""))

    a.publish("tcp://localhost:5553", "drive", get_data, 1, True)

    time.sleep(10)
    a.quit()
