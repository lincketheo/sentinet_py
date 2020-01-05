from sentinet.message.Message import *
import threading

class cmd_vel(Data_Message):
    def __init__(self):
        super().__init__()
        self.lin = 0.0
        self.ang = 0.0

        self.push_data(0.0, 4, FLOAT)
        self.push_data(0.0, 4, FLOAT)
        self.to_wire()

        self.lock = threading.Lock()

    def prepare(self):
        self.set_data(self.lin, 4, FLOAT, 0)
        self.set_data(self.ang, 4, FLOAT, 1)

    def safe_update(self, lin, ang):
        self.lock.acquire()
        try:
            self.lin = lin
            self.ang = ang
        finally:
            self.lock.release()

    def get(self):
        lin = 0.0
        ang = 0.0

        self.lock.acquire()
        try:
            lin = self.lin
            ang = self.ang
        finally:
            self.lock.release()

        return lin, ang

    def to_string(self):
        self.prepare()
        return bytes(self.message)

    def get_size(self):
        return self.header["BYTE_LENGTH"]

    def from_wire(self, incomming_message):
        self.parse_from_similar_message(incomming_message)

        l = struct.unpack('f', self.get_data(0))[0]
        a = struct.unpack('f', self.get_data(1))[0]

        self.safe_update(l, a)
        return self.get()


class path_state(Data_Message):
    def __init__(self):
        super().__init__()
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.v = 0.0
        self.w = 0.0

        self.push_data(0.0, 4, FLOAT)
        self.push_data(0.0, 4, FLOAT)
        self.push_data(0.0, 4, FLOAT)
        self.push_data(0.0, 4, FLOAT)
        self.push_data(0.0, 4, FLOAT)

        self.to_wire()

        self.lock = threading.Lock()

    def prepare(self):
        self.lock.acquire()
        try:
            self.set_data(self.x, 4, FLOAT, 0)
            self.set_data(self.y, 4, FLOAT, 1)
            self.set_data(self.theta, 4, FLOAT, 2)
            self.set_data(self.v, 4, FLOAT, 3)
            self.set_data(self.w, 4, FLOAT, 4)
        finally:
            self.lock.release()

    def safe_update(self, x, y, theta, v, w):
        self.lock.acquire()
        try:
            self.x = x
            self.y = y
            self.theta = theta
            self.v = v
            self.w = w
        finally:
            self.lock.release()

    def get(self):
        x = 0.0
        y = 0.0
        theta = 0.0
        v = 0.0
        w = 0.0

        self.lock.acquire()
        try:
            x = self.x
            y = self.y
            theta = self.theta
            v = self.v
            w = self.w
        finally:
            self.lock.release()

        return x, y, theta, v, w

    def to_string(self):
        self.prepare()
        return bytes(self.message)

    def get_size(self):
        return self.header["BYTE_LENGTH"]

    def from_wire(self, incomming_message):
        self.parse_from_similar_message(incomming_message)

        x = struct.unpack('f', self.get_data(0))[0]
        y = struct.unpack('f', self.get_data(1))[0]
        theta = struct.unpack('f', self.get_data(2))[0]
        v = struct.unpack('f', self.get_data(3))[0]
        w = struct.unpack('f', self.get_data(4))[0]

        print(x, y, theta, v, w)

        self.safe_update(x, y, theta, v, w)
        return self.get()

class ping_message(Ping_Message):
    def __init__(self):
        super().__init__()
        self.type = 0
        self.code = 0
        self.excess = 0
        self.lock = threading.Lock()
        
    def prepare(self):
        self.lock.acquire()
        try:
            self.set_type(self.type)
            self.set_code(self.code)
            self.set_excess(self.excess)
        finally:
            self.lock.release()

    def get(self):
        t = 0
        c = 0
        e = 0
        self.lock.acquire()
        try:
            t = self.type
            c = self.code
            e = self.excess
        finally:
            self.lock.release()

        return t, c, e

    def safe_update(self, type_, code_, excess_):
        self.lock.acquire()
        try:
            self.type = type_
            self.code = code_
            self.excess = excess_
        finally:
            self.lock.release()

    def to_string(self):
        self.prepare()
        return bytes(self.message)

    def get_size(self):
        return self.header_size

    def from_wire(self, incomming_message):
        self.message = bytearray(incomming_message)

        t = self.message[2]
        c = int.from_bytes(bytes(self.message[3:5]), ENDIAN)
        e = int.from_bytes(bytes(self.message[7:15]), ENDIAN)

        self.safe_update(t, c, e)
        return self.get()
















