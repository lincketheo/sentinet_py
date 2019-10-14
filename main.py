from sentinet.curmt import KermitControlModule
from time import sleep

if __name__ == '__main__':
    a = KermitControlModule()

    a.start_kermit()

    sleep(4)

    a.set_linear(7.0)

    sleep(5)

    a.set_angular(5.6)

    a.quit_kermit()
