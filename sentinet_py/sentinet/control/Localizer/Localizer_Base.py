from multiprocessing import Pipe
from abc import ABC, abstractmethod
from sentinet.curmt import KermitControlModule
import numpy as np

#Template for localizers

class LocalizerBase(ABC):
	def __init__(self, pipe, sensors):
		self.pipe = pipe
		self.sensors = sensors
		self.position = np.full((3,1),None)
		self.velocity = np.full((3,1),None)
		self.ang_position = np.full((3,1),None)
		self.ang_velocity = np.full((3,1),None)

	def pipe_value(self,value): #helper function to send value to state machine
		self.pipe.send(value)

	@abstractmethod
	def filter(self):
		pass

	@abstractmethod
	def dynamics_model(self):
		pass

class SensorBase(ABC): #Template for a sensor
	def __init__(self):
		self.ControlModule = KermitControlModule()

	def get_data(self):
		return self.data

	@abstractmethod
	def callback(self):
		pass

	@abstractmethod
	def sensor_model(self):
		pass
