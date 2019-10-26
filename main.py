from sentinet.curmt import KermitControlModule
from time import sleep

def temporaryfunc(val1: float, val2: float):
    print(val1, val2)

if __name__ == '__main__':
    a = KermitControlModule()
    a.set_cmd_vel_get_data(temporaryfunc)

    a.start_kermit()

    sleep(4)

    a.set_linear(7.0)

    sleep(5)

    a.set_angular(5.6)
    
    sleep(4)

    a.quit_kermit()
