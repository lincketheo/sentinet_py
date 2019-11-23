from sentinet.control.Localizer.Localizer_Base import LocalizerBase, SensorBase
from threading import Lock
from sentinet.curmt import KermitControlModule
import numpy as np
import time




# Geometric point offset in actual sensor classes, not localizer
# Individual instances for each imu


# Test basic class for operation for a localizer.
class DummyLocalizer(LocalizerBase):
	##
	# @brief Initializes the DummyLocalizer class. If no values are set, it will just set values to default.
	#
	# @param pipe The pathway the information is traveling through.
	# @param sensors The actual name of the sensor that the is sending information.
	def __init__(self, pipe, sensors):
		super().__init__(pipe, sensors)
		if None in self.position:
			self.position.fill(0.0)
		if None in self.velocity:
			self.velocity.fill(0.0)
		if None in self.ang_position:
			self.ang_position.fill(0.0)
		if None in self.ang_velocity:
			self.ang_velocity.fill(0.0)
		self.pipe_value([self.position, self.ang_position])
		self.run_localizer()
			
	# @brief Stores data for the localizer for passing to the pipe.
	def filter(self):
		# integrate sensor data forward
		for sensor_name, sensor in zip(self.sensors.keys(),self.sensors.values()):
			if sensor_name is 'DummySensor':
				cmd_vel = sensor.get_data()
				throttle = cmd_vel[0]
				turn_ratio = cmd_vel[1]
				self.velocity = np.array([np.cos(float(self.ang_position[0]))*throttle*drive_gain, np.sin(float(self.ang_position[0]))*throttle*drive_gain, 0])
				self.ang_velocity = np.array([np.pi*turn_ratio*turn_gain, 0, 0])
				self.position = self.position + np.array([self.velocity[0]*time_step, self.velocity[1]*time_step, self.velocity[2]*time_step])
				self.ang_position = self.ang_position + np.array([self.ang_velocity[0]*time_step, self.ang_velocity[1]*time_step, self.ang_velocity[2]*time_step])
		self.pipe_value([self.position, self.ang_position])

	def run_localizer(self):
		while True:
			try:
				if self.read_pipe() == 'fin':
					self.end_localizer()
					exit()
				else:
					self.filter()
			except KeyboardInterrupt:
				exit()
	
	def dynamics_model(self):
		return 0

class ImuSensor(SensorBase):
    def __init__(self, x_pos, x_offset, y_offset):
        self.data = {
            "x_acc": 0,
			"y_acc": 0,
            "ang_acc": 0,
            "x_vel": 0,
            "y_vel": 0,
            "ang_vel": 0
            }
        self.x_offset = x_offset
        self.y_offset = y_offset
        self.lock = Lock()
        self.ControlModule = KermitControlModule(requesting=True)
        self.ControlModule.set_data_callback(self.callback)

    # Theo and Chris: talk to about spinning up sensors, as well as information
	# Figure out how to differentiate instances
    # rate of update?
    # Clock Time Ideas?
    def start_sensor(self):
        self.ControlModule.start_kermit()

    def quit_sensor(self):
        self.ControlModule_quit.kermit()

    def get_data(self):
        return self.data

    def reset_velocity(self, x_vel, y_vel, ang_vel):
        self.data[x_vel] = x_vel
        self.data[y_vel] = y_vel
        self.data[ang_vel] = ang_vel

    def callback(self, x_acc: float, y_acc: float, theta_acc: float):
		try:
            #   Print python timestamp
			self.lock.acquire()
			self.data = self.sensor_model(x_acc, y_acc, theta_acc)
			self.lock.release()
		except KeyboardInterrupt:
			exit()
		return

	# integrate to get velocity
    def sensor_model(self, x_acc: float, y_acc: float, ang_acc: float):
        time = time.time()
        delta_time = time.time()-self.time
        self.time = time
        return {
            "x_acc": x_acc,
			"y_acc": y_acc,
            "ang_acc": theta_acc,
            "x_vel": self.data["x_vel"]+delta_time*((x_acc+self.data["x_acc"])/2),
            "y_vel": self.data["y_vel"]+delta_time*((y_acc+self.data["y_acc"])/2),
            "ang_vel": self.data["ang_vel"]+delta_time*((ang_acc+self.data["ang_acc"])/2),
            }


class AprilTags(SensorBase):
   def __init__(self, x_offset, y_offset, x_pos, y_pos, heading):
        self.data = {
            "x_pos": x_pos or 0,
            "y_pos": y_pos or 0,
            "heading": heading or 0,
            "x_vel": 0,
            "y_vel": 0,
			"ang_vel": 0,
        }
        self.x_offset = x_offset
        self.y_offset = y_offset
        self.lock = Lock()
		self.time = time.time()
        self.ControlModule = KermitControlModule(requesting=True)
        self.ControlModule.set_data_callback(self.callback)

    #   April Tags Camera? - how to start
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

    def callback(self, x_pos: float, y_pos: float, heading: float):
        try:
            #   Print python timestamp
			self.lock.acquire()
			self.data = self.sensor_model(x_pos, y_pos, heading)
			self.lock.release()
		except KeyboardInterrupt:
			exit()
		return

    #   Derive linear/angular velocity
    def sensor_model(self, x_pos: float, y_pos: float, heading: float):
        time = time.time()
        delta_time = time.time()-self.time
        self.time = time
        return {
            "x_pos": x_pos,
            "y_pos": y_pos,
            "heading": heading,
            "x_vel": (x_pos - self.data["x_pos"]) / delta_t,
            "y_vel": (y_pos - self.data['y_pos']) / delta_t,
			"ang_vel": (heading-self.data['heading']) / delta_t
        }

# All communication running on publisher/subscriber model
# Colmon filter to combine velocities in localizer
