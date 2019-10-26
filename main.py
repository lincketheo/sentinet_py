from sentinet.curmt import KermitControlModule
from time import sleep

def temporaryfunc(val1: float, val2: float):
    print("doing stuff")
    print(val1, val2)

def temporaryfunc2():
    print("Doing other stuff")
    return 4.5, 6.7

if __name__ == '__main__':
    a = KermitControlModule()
    a.set_cmd_vel_get_data(temporaryfunc2)
    a.set_data_callback(temporaryfunc)
   
    a.start_kermit()

    sleep(4)

    sleep(5)

    sleep(4)

    a.quit_kermit()
