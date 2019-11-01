from multiprocessing import Process, Pipe
from time import time
from abc import ABC, abstractmethod
from sentinet.core.control.ControlClient import ControlClient

class ActionStateBase(ABC):
	def __init__(self, pipe): #initialize action state with comm pipe to state machine
		self.pipe = pipe
		while self.get_state() is None:
			self.get_state()
		self.init_control_module()

	def set_data(self, data):
		self.data = data

	def get_data(self): #retrieves the data relevant from this state
		return self.data

	def get_state(self): #get system state from state machine
		if self.pipe.poll():
			return self.pipe.recv()
		else:
			return None

	def pipe_value(self, value): #helper function to send value to state machine
		self.pipe.send(value)

	@abstractmethod
	def execute(self): #abstract method to execute state functions
		pass

	@abstractmethod
	def init_control_module(self):
		pass

	@abstractmethod
	def end_state(self): #end state process
		pass

class StateMachineBase(ABC):
	def __init__(self, alphabet, state_list, t_max, localizer, sensors, init_state=None):
		#alphabet defines system state variables
		#state_list contains list of state objects to be executed
		#t_max is maximum run time in seconds
		#init_state is an optional initial system state, if none will be set to first update state call
		self.init_time = time()
		self.t_max = t_max
		self.state_list = state_list
		if init_state is not None:
			self.state = {alphabet[i]:init_state[i] for i in range(len(alphabet))}
		else:
			self.state = {alphabet[i]: None for i in range(len(alphabet))}
		self.start_localizer(localizer, sensors)

	@abstractmethod
	def transition_law(self):
		#transition_law is a callable function that takes in (current_action_state,system_state) and outputs
		# the next action state. Where current_action_state is an integer corresponding to the position in the
		# state list
		pass

	@abstractmethod
	def update_system_state(self): #abstract method to update system state
		pass

	@abstractmethod
	def init_system(self): #abstract method to initialize state machine
		pass

	@abstractmethod
	def run_SM(self):#abstract method to run state machine
		pass

	def execute_state(self,state): #set up action state with communication pipe
		machine_conn, state_conn=Pipe()
		self.pipe = machine_conn
		s=Process(target=state,args=(state_conn,))
		s.start()

	def start_localizer(self, localizer, sensors):
		machine_conn, loc_conn = Pipe()
		self.loc_pipe = machine_conn
		s=Process(target=localizer, args=(loc_conn, sensors))

	def read_pipe(self): #helper function to read pipe in a non-blocking way
		if self.pipe.poll():
			return self.pipe.recv()
		else:
			return None

	def read_loc_pipe(self):
		if self.loc_pipe.poll():
			return self.loc_pipe.recv()
		else:
			return None

	def pipe_state(self): #send system state to action state
		self.pipe.send(self.state)
