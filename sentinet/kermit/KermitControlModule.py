from sentinet import ControlClient, pub_params, sub_params
from sentinet.message import cmd_vel, path_state, ping_message
from sentinet.common import *

class KermitControlModule(ControlClient):

    def __init__(self):
        super().__init__(to_bind_addr(controller["ADDRESS"]))

        self.command_data_out = ping_message()
        self.command_data_in = ping_message()

        # Params
        self.cmd_vel = pub_params()
        self.cmd_vel.period = 1.0
        self.cmd_vel.sock_addr = to_conn_addr(controller["CMD_VEL"])
        self.cmd_vel.start_on_creation = True
        self.cmd_vel.topic = topics["cmd_vel_topic"]
        self.cmd_vel.get_data = self.__cmd_vel_get_data
        
        self.cmd_msg = cmd_vel()
        

        self.state = sub_params()
        self.state.callback = self.__state_set_state 
        self.state.sock_addr = to_conn_addr(controller["LOCALIZER"])
        self.state.topic = topics["localizer_topic"]
        self.state.start_on_creation = True

        self.path_state = path_state()

        self.init_self_client()

        self.spin_publisher(self.cmd_vel)
        self.spin_subscriber(self.state)

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.v = 0.0
        self.w = 0.0

    
    def ping(self, address, type_: int, code: int, excess: int):
        self.command_data_out.safe_update(type_, code, excess)
        resp = self.request_concurrent(address, (self.command_data_out.to_string()))
        if(resp == b'Error'):
            return resp
        self.command_data_in.from_wire(resp)
        return self.command_data_out.get()

    # Stop kermit
    def quit_kermit(self):
        self.quit()

    def safe_update(self, lin, ang):
        lin_ = float(lin)
        ang_ = float(ang)
        self.cmd_msg.safe_update(lin_, ang_)

    def __cmd_vel_get_data(self):
        self.cmd_msg.safe_update(4.5, 6.7)
        return self.cmd_msg.to_string()

    def __state_set_state(self, val):
        self.path_state.from_wire(val)
        x, y, theta, v, w = self.path_state.get()
        self.x = x
        self.y = y
        self.theta = theta
        self.v = v
        self.w = w

    def safe_get_state(self):
        return self.x, self.y, self.theta, self.v, self.w
