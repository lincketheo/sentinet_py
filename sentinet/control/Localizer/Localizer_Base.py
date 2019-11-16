from multiprocessing import Pipe
from abc import ABC, abstractmethod
from sentinet.curmt import KermitControlModule
import numpy as np

# Template for localizers


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

    def pipe_value(self, value):  # helper function to send value to state machine
        self.pipe.send(value)

    def end_localizer(self):
        for sensor in self.sensors.values():
            sensor.quit_sensor()

    def read_pipe(self):  # helper function to read pipe in a non-blocking way
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


class SensorBase(ABC):  # Template for a sensor
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

# Viable?
# Useful when april tags not visible
# Maybe reset every n seconds using april tags
	
# Geometric point offset in actual sensor classes, not localizer
# Individual instances for each imu


class ImuSensor(SensorBase):
   def __init__(self, x_pos, x_offset, y_offset):
        self.data = {
            "x_acc": 0,
			"y_acc": 0,
            "theta_acc": 0,
            "x_vel": 0,
            "y_vel": 0,
            }
        self.x_offset = x_offset
        self.y_offset = y_offset
        self.lock = Lock()
        self.ControlModule = KermitControlModule(requesting=True)
        self.ControlModule.set_data_callback(self.callback)

    # Theo and Chris: talk to about spinning up sensors, as well as information
	# Figure out how to differentiate instances
    # rate of update?
    def start_sensor(self):
        self.ControlModule.start_kermit()

    def quit_sensor(self):
        self.ControlModule_quit.kermit()

    def get_data(self):
        return self.data

    def callback(self, x_acc, y_acc, angle_acc):
		self.data[x_acc] = x_acc
        self.data[y_acc] = y_acc
        self.data[angle_acc] = angle_acc
		sensor_model()

	# integrate to get velocity
    def sensor_model(self):
        pass


# Getting x, y, theta data
class AprilTags(SensorBase):
   def __init__(self, x_offset, y_offset, x_pos, y_pos, heading):
        self.data = {
            "x_pos": x_pos or 0,
            "y_pos": y_pos or 0,
            "heading": heading or 0
        }
        self.x_offset = x_offset
        self.y_offset = y_offset
        self.lock = Lock()
        self.ControlModule = KermitControlModule(requesting=True)
        self.ControlModule.set_data_callback(self.callback)

    # April Tags Camera? - how to start
    # rate of update?
    def start_sensor(self):
        rospy.init_node('Bot_Position', anonymous=True)
        t = tf2_ros.BufferCore(rospy.Duration(1))
    	tfBuffer = tf2_ros.Buffer()
    	listener = tf2_ros.TransformListener(tfBuffer)

       self.ControlModule.start_kermit()

    def quit_sensor(self):
        self.ControlModule_quit.kermit()

    def get_data(self):
        return self.data

    def callback(self, x_pos, y_pos, heading):
        self.data[x_pos] = x_pos
        self.data[y_pos] = y_pos
        self.data[heading] = heading
		sensor_model()

	# derive velocity
    def sensor_model(self):
        pass

# All communication running on publisher/subscriber model
# Common filter to combine velocities in localizer