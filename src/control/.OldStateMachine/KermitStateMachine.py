from .StateMachine import *

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


class InitState(State):
    def __init__(self, state_label: int, state_desc: str):
        super().__init__(state_label, state_desc)
        self.add_transition_function("lambda1", self.enter_drive_mode)
        self.add_transition_function("kill", self.kill_switch)

    def enter_drive_mode(self):
        return self.alphabet.get_var("start_flag")

    def kill_switch(self):
        return self.alphabet.get_var("kill_flag")

class DriveState(State):
    def __init__(self, state_label: int, state_desc: str):
        super().__init__(state_label, state_desc)
        self.add_transition_function("lambda2", self.enter_mine_mode)
        self.add_transition_function("lambda3", self.enter_dump_mode)
        self.add_transition_function("kill", self.kill_switch)

    def enter_mine_mode(self):
        return self.alphabet.get_var("mining")

    def enter_dump_mode(self):
        return self.alphabet.get_var("dumping")

    def kill_switch(self):
        return self.alphabet.get_var("kill_flag")

class MineState(State):
    def __init__(self, state_label: int, state_desc: str):
        super().__init__(state_label, state_desc)
        self.add_transition_function("lambda4", self.enter_drive_mode)
        self.add_transition_function("kill", self.kill_switch)

    def enter_drive_mode(self):
        return self.alphabet.get_var("moving")

    def kill_switch(self):
        return self.alphabet.get_var("kill_flag")

class DumpState(State):
    def __init__(self, state_label: int, state_desc: str):
        super().__init__(state_label, state_desc)
        self.add_transition_function("lambda5", self.enter_drive_mode)
        self.add_transition_function("kill", self.kill_switch)

    def enter_drive_mode(self):
        return self.alphabet.get_var("moving")

    def kill_switch(self):
        return self.alphabet.get_var("kill_flag")

class KillState(State):
    def __init__(self, state_label: int, state_desc: str):
        super().__init__(state_label, state_desc)

