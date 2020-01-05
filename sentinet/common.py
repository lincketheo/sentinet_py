# Topics on which to do a transaction
topics = {"cmd_vel_topic" : "curmt://cmd_vel", "localizer_topic" : "curmt://state_localizer"}

# Proxies
localizer = {"FRONT_ADDRESS" : 5452, "BACK_ADDRESS" : 5454}
command_velocity = {"FRONT_ADDRESS" : 5470, "BACK_ADDRESS" : 5472}

# Kernel Attributes
kernel = {"ADDRESS" : 5450, "LOCALIZER" : localizer["FRONT_ADDRESS"], "CMD_VEL" : command_velocity["BACK_ADDRESS"]}

# Controller Attributes
controller = {"ADDRESS" : 5456, "LOCALIZER" : localizer["BACK_ADDRESS"], "CMD_VEL" : command_velocity["FRONT_ADDRESS"]}


def to_conn_addr(val):
    return "tcp://localhost:" + str(val)

def to_bind_addr(val):
    return "tcp://*:" + str(val)



# Some default callbacks for control client
def get_data_default():
    return "default"

def sub_default_callback(data):
    print("Subscriber recieved ", val)

def serve_default_callback(val):
    print("Server recieved ", val)
    return val + " response"

def req_callback(val):
    print("Requester recieved val " + val)


"""
Params for control client (same as C++)
"""
class pub_params:
    def __init__(self):
        self.sock_addr = "NOADDR"
        self.get_data = get_data_default
        self.topic = ""  
        self.period = 1 
        self.start_on_creation = True

class sub_params:
    def __init__(self):
        self.sock_addr = "NOADDR"
        self.callback = sub_default_callback
        self.topic = ""
        self.start_on_creation = True

class serve_params:
    def __init__(self):
        self.sock_addr = "NOADDR"
        self.callback = serve_default_callback
        self.start_on_creation = True

class req_params():
    def __init__():
        self.sock_addr = "NOADDR"
        self.get_data = get_data_default
        self.callback = req_callback
        self.start_on_creation = True

