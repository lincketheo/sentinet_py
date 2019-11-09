from multiprocessing import Pipe
from abc import ABC, abstractmethod
from sentinet.curmt import KermitControlModule
import numpy as np

#Template for localizers

class LocalizerBase(ABC):
	def __init__(self, pipe, sensors):
		self.pipe = pipe
		self.sensors = sensors
		self.position = np.array([None, None, None])
		self.velocity = np.array([None, None, None])
		self.ang_position = np.array([None, None, None])
		self.ang_velocity = np.array([None, None, None])

		for sensor in self.sensors.values():
			sensor.start_sensor()

	def pipe_value(self,value): #helper function to send value to state machine
		self.pipe.send(value)

	def end_localizer(self):
		for sensor in self.sensors.values():
			sensor.quit_sensor()

	def read_pipe(self): #helper function to read pipe in a non-blocking way
		return self.pipe.recv()

	@abstractmethod
	def run_localizer(self):
		pass

	@abstractmethod
	def filter(self):
		pass

	@abstractmethod
	def dynamics_model(self):
		pass

class SensorBase(ABC): #Template for a sensor
	def __init__(self):
		pass

	@abstractmethod
	def start_sensor(self):
		pass

	@abstractmethod
	def quit_sensor(self):
		pass
		
	def get_data(self):
		return self.data

	@abstractmethod
	def callback(self):
		pass

	@abstractmethod
	def sensor_model(self):
		pass
	
class ImuSensor(SensorBase)
	def __init__(self, pos):
		self.x_acc=0
		self.y_acc=0
		self.angle_acc=0
		self.pos=pos
		self.lock = Lock()
		self.ControlModule = KermitControlModule(requesting=True)
		self.ControlModule.set_data_callback(self.callback)
	
	def start_sensor(self):
		self.ControlModule.start_kermit()

	def quit_sensor(self):
		self.ControlModule_quit.kermit()
		
	def get_data(self):
		return 

	def callback(self, x: float, y: float, angle):
		p
	
	def sensor_model(self):
		pass
	def start_sensor()
...
self.data_callback_ataxgs( x float, y float, y_angle float)
self.data_callback_imu(x_acc float, y_acc float, y_angle_acc float)
...
	
