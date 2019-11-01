from sentinet.control.StateMachine.State_Machine_Base import StateMachineBase, ActionStateBase
from sentinet.control.Localizer import CommsTestLocalizer
from sentinet.curmt.KermitControlModule import KermitControlModule
import numpy as np
import math
from multiprocessing import Process, Pipe
from time import time, sleep

DISCRETIZATION_SIZE=100
ATTRACTOR=[0,0]
PATH_TOL=0.05 #meters
CHECKSUM="CHECKSUM"
# todo: better transition law
#		verify requester flags
#		dynamic mapping in movement states
class RMT_SM(StateMachineBase):
	mining_zone=[[10.0,10.0],[10.0,10.0]]
	dumping_zone=[[15.5,15.5],[-10.0,-10.0]]
	def __init__(self, alphabet, state_list, t_max, localizer, sensors, init_state=None):
		super().__init__(alphabet, state_list, t_max, localizer, sensors, init_state=init_state)
		self.init_system()
	
	def init_system(self): #start in initial state
		self.curr_state=0
		self.execute_state(state_list[0])
		self.run_SM()
	
	#Defines what is legal for state transfer between states to make sure
	def transistion_law(self): #transition law defined in RMT SM Definition, see Drive
		run_time=time()-self.init_time
		# Soft Exit Conditions
		if self.state['s'] == 0:
			return 5
		if run_time >= self.t_max:
			return 5

		if self.curr_state == 0: # Transition from state init state
			if (self.state['x'] is None 
				and self.state['y'] is None 
				and self.state['th'] is None 
				and self.state['m'] is False 
				and self.state['d'] is False 
				and self.state['f'] is False):
				return 0

			elif (self.state['x'] is not None 
				and self.state['y'] is not None 
				and self.state['th'] is not None 
				and self.state['a'] is True 
				and self.state['v'] is False 
				and self.state['m'] is False 
				and self.state['d'] is False 
				and self.state['f'] is False):
				return 1

		elif self.curr_state == 1: # Transition from mv2mine
			if ((self.state['x'] < self.mining_zone[0][0] or self.state['x'] > self.mining_zone[0][1])
				and (self.state['y'] < self.mining_zone[1][0] or self.state['y'] > self.mining_zone[1][1]) 
				and self.state['a'] is True  
				and self.state['m'] is False 
				and self.state['d'] is False 
				and self.state['f'] is False):
				return 1
			elif ((self.state['x'] > self.mining_zone[0][0] and self.state['x'] < self.mining_zone[0][1])
				and (self.state['y'] > self.mining_zone[1][0] and self.state['y'] < self.mining_zone[1][1])
				and self.state['a'] is True
				and self.state['m'] is False
				and self.state['d'] is False
				and self.state['f'] is False
				and self.state['v'] is False):
				return 2


		elif self.curr_state == 2: # Transition from mine
			if ((self.state['x'] > self.mining_zone[0][0] and self.state['x'] < self.mining_zone[0][1])
				and (self.state['y'] > self.mining_zone[1][0] and self.state['y'] < self.mining_zone[1][1])
				and self.state['a'] is True
				and self.state['m'] is True
				and self.state['d'] is False
				and self.state['f'] is False
				and self.state['v'] is False):
				return 2

			elif ((self.state['x'] > self.mining_zone[0][0] and self.state['x'] < self.mining_zone[0][1])
				and (self.state['y'] > self.mining_zone[1][0] and self.state['y'] < self.mining_zone[1][1])
				and self.state['a'] is True
				and self.state['m'] is False
				and self.state['d'] is False
				and self.state['f'] is True
				and self.state['v'] is False):
				return 3
		elif self.curr_state == 3: # Transition from mv2dump
			if ((self.state['x'] < self.dumping_zone[0][0] or self.state['x'] > self.dumping_zone[0][1])
				and (self.state['y'] < self.dumping_zone[1][1] or self.state['y'] > self.dumping_zone[1][1])
				and self.state['a'] is True
				and self.state['m'] is False
				and self.state['d'] is False
				and self.state['f'] is True):
				return 3
			elif ((self.state['x'] > self.dumping_zone[0][0] and self.state['x'] < self.dumping_zone[0][1])
				and (self.state['y'] > self.dumping_zone[1][0] and self.state['y'] < self.dumping_zone[1][1])
				and self.state['a'] is True
				and self.state['m'] is False
				and self.state['d'] is False
				and self.state['f'] is True
				and self.state['v'] is False):
				return 4

		elif self.curr_state == 4: # Transition from dump
			if ((self.state['x'] > self.dumping_zone[0][0] and self.state['x'] < self.dumping_zone[0][1])
				and (self.state['y'] > self.dumping_zone[1][0] and self.state['y'] < self.dumping_zone[1][1])
				and self.state['a'] is True
				and self.state['m'] is False
				and self.state['d'] is True
				and self.state['f'] is True
				and self.state['v'] is False):
				return 4
			elif ((self.state['x'] > self.dumping_zone[0][0] and self.state['x'] < self.dumping_zone[0][1])
				and (self.state['y'] > self.dumping_zone[1][0] and self.state['y'] < self.dumping_zone[1][1])
				and self.state['a'] is True
				and self.state['m'] is False
				and self.state['d'] is False
				and self.state['f'] is False
				and self.state['v'] is False):
				return 1

	def update_system_state(self): #update sys_state from localizer
		pos=self.read_loc_pipe()
		if pos is not None:
			self.state['x']=pos[0][0]
			self.state['y']=pos[0][1]
			self.state['th']=pos[1][0]

	def run_SM(self): #master run loop
		while True:
			self.update_system_state()
			pipe_check=self.read_pipe()
			if pipe_check is not None:
				keys=pipe_check.keys()
				if 'fin' in keys:
					self.localizer.end_localizer()
					raise SystemExit
				else:
					for key in keys:
						self.state[key]=pipe_check[key]
			new_state=self.transition_law()
			if new_state==self.curr_state:
				self.pipe_state()
			else:
				self.curr_state=new_state
				self.execute_state(self.curr_state)
				self.pipe_state()

class mv2mine(ActionStateBase): #move to mining position action state
	def __init__(self, pipe): #starts using the init function to a pipe listed to it super class
		super().__init__(pipe)
		#self.get_map()

	def init_control_module(self): #add dynamic mapping reciever here
		self.ControlModule = KermitControlModule(publishing=True)
		self.ControlModule.set_cmd_vel_get_data(self.cmd_vel_callback)
		self.ControlModule.set_data_callback(self.mapping_callback)
		self.ControlModule.start_kermit()

	def cmd_vel_callback(self):
		cmd_vel = self.get_data()
		return cmd_vel[0], cmd_vel[1]

	def get_map(self):
		pass

	def mapping_callback(self, message_string):
		pass

	def execute(self): # executes minning operations
		self.target = self.select_target_zone()
		self.path = self.determine_path()
		self.run_PD()
		self.end_state()

	def end_state(self): # ends the mining process
		self.ControlModule.quit_kermit()

	def select_target_zone(self): #select target pos from zone as np array, zone boundaries hard coded from reqs
		pass

	def determine_path(self): #Bezier Curve Path Generator
		self.path,self.dpath = Bez_Cur([self.state['x'], self.state['y']], self.target, ATTRACTOR)

	def run_PD(self): #while loop run of PD controller
		self.pipe_value(dict(moving=True))
		self.np_pos = np.array([self.state['x'], self.state['y']])
		self.vel = np.array([0,0])
		self.pos_diff_norm = np.linalg.norm(self.np_pos - self.target)
		while self.pos_diff_norm > PATH_TOL:
			self.set_data(GLPDC(self.path, self.dpath, self.np_pos, self.vel, 0))
			if self.get_state() is not None:
				self.state = self.get_state()
			self.vel = np.array([[self.state['x'],self.state['y']]])-self.np_pos
			self.np_pos = np.array([self.state['x'],self.state['y']])
			self.pos_diff_norm = np.linalg.norm(self.np_pos-self.target)
		self.set_data([0,0])
		sleep(0.05)
		self.pipe_value(dict(moving=False))

class mine(ActionStateBase): #mining action state
	def __init__(self,pipe):
		super().__init__(pipe)

	def init_control_module(self):
		self.ControlModule = KermitControlModule(requesting=True)
		self.ControlModule.start_kermit()

	def mine_requester(self):
		self.ControlModule.request(8,1<<2,1<<1)

	def execute(self):
		self.pipe_value({'mining': True})
		self.mine_requester()
		self.pipe_value({'mining': False})
		self.end_state()

	def end_state(self):
		self.ControlModule.quit_kermit()


class mv2dump(ActionStateBase): #moving to dumping zone mining state
	def __init__(self,pipe):
		super().__init__(pipe)
		#self.get_map()

	def execute(self):
		self.target = self.select_target_zone()
		self.path = self.determine_path()
		self.run_PD()
		self.end_state()

	def end_state(self):
		self.ControlModule.quit_kermit()
	
	def init_control_module(self): #add dynamic mapping reciever here
		self.ControlModule = KermitControlModule(publishing=True)
		self.ControlModule.set_cmd_vel_get_data(self.cmd_vel_callback)
		self.ControlModule.set_data_callback(self.mapping_callback)
		self.ControlModule.start_kermit()

	def cmd_vel_callback(self):
		cmd_vel = self.get_data()
		return cmd_vel[0], cmd_vel[1]

	def get_map(self):
		pass

	def mapping_callback(self, message_string):
		pass

	def select_target_zone(self): #hard coded return point based on reqs
		pass

	def determine_path(self): #Bezier curve path generator
		self.path,self.dpath = Bez_Cur([self.state['x'], self.state['y']], self.target, ATTRACTOR)

	def run_PD(self): #while loop for PD controller
		self.pipe_value(dict(moving=True))
		self.np_pos = np.array([self.state['x'], self.state['y']])
		self.vel = np.array([0,0])
		self.pos_diff_norm = np.linalg.norm(self.np_pos - self.target)
		while self.pos_diff_norm > PATH_TOL:
			self.set_data(GLPDC(self.path, self.dpath, self.np_pos, self.vel, 1))
			if self.get_state() is not None:
				self.state = self.get_state()
			self.vel = np.array([[self.state['x'], self.state['y']]]) - self.np_pos
			self.np_pos=np.array([self.state['x'], self.state['y']])
			self.pos_diff_norm=np.linalg.norm(self.np_pos - self.target)
		self.set_data([0,0])
		sleep(0.05)
		self.pipe_value(dict(moving=False))

class dump(ActionStateBase): #Class for the to dump state.
	def __init__(self,pipe):
		super().__init__(pipe)

	def init_control_module(self):
		self.ControlModule = KermitControlModule(requesting=True)
		self.ControlModule.start_kermit()

	def dump_requester(self):
		self.ControlModule.request(8,1<<1,1<<1)

	def execute(self):
		self.pipe_value({'dumping': True})
		self.dump_requester()
		self.pipe_value({'dumping': False})
		self.end_state()

	def end_state(self):
		self.ControlModule.quit_kermit()


class init_state(ActionStateBase): #initialization state
	def __init__(self,pipe): #initializes the initializer
		super().__init__(pipe)
	
	def init_control_module(self):
		self.ControlModule = KermitControlModule(requesting=True)
		self.ControlModule.start_kermit()

	def cam_requester(self):
		self.ControlModule.request(1,1,1)

	def execute(self):
		self.cam_requester()
		self.pipe_value({'a': True})
		self.end_state()

	def end_state(self):
		self.ControlModule.quit_kermit()



class soft_exit(ActionStateBase): #planned soft exit state
	def __init__(self,pipe): #initializes the soft exit state
		super().__init__(pipe)

	def end_mine_requester(self):
		self.ControlModule.request(1,1,1)

	def dump_requester(self):
		self.ControlModule.request(1,1,1)

	def kill_requester(self):
		self.ControlModule.request(1,1,1)

	def execute(self):
		sleep(0.05)
		self.pipe_value({'moving': False})
		self.end_mine_requester()
		self.pipe_value({'mining': False, 'dumping': True})
		self.dump_requester()
		self.pipe_value({'dumping': False, 'full': False})
		self.kill_requester()
		self.pipe_value(dict(fin=True))
		self.end_state()

	def cmd_vel_callback(self):
		return 0, 0

	def init_control_module(self):
		self.ControlModule = KermitControlModule(requesting=True, publishing=True)
		self.ControlModule.set_cmd_vel_get_data(self.cmd_vel_callback)
		self.start_kermit()

	def end_state(self):
		self.ControlModule.quit_kermit()



def Bez_Cur(Start,End,Attractor,weight):
	#Start End and Attractor must be x,y pairs, weight [-1,1]
	t=np.arange(DISCRETIZATION_SIZE)/DISCRETIZATION_SIZE
	M=np.zeros([2,len(t)])
	#Generate bezier curve for travel
	for i in range(len(t)):
		s=1-t[i]
		div=s**2+2*weight*s*t[i]+t[i]**2
		M[0][i]=(((s**2)*Start[0]+2*weight*s*t[i]*Attractor[0]+(t[i]**2)*End[0])/div)
		M[1][i]=(((s**2)*Start[1]+2*weight*s*t[i]*Attractor[1]+(t[i]**2)*End[1])/div)

	#generate finite difference velocities
	dM=np.zeros([2,len(t)-1])
	dM[0]=[x-M[0][i-1] for i,x in enumerate(M[0])][1:]
	dM[1]=[x-M[1][i-1] for i,x in enumerate(M[1])][1:]
	return M,dM

def GLPDC(path,pHeadings,position,velocity,backwards):
	#path as 2 by Dis_Size set of discrete path points
	#pHeadings as 2xDis_size-1 set of path headings
	#position as 3x1 x,y,th vector
	#velocity as 2x1 dx,dy vector

	#find closest position on path to determine parameter value [0,1]
	i=np.argmin((path[0]-position[0])**2+(path[1]-position[1])**2)
	t=i/DISCRETIZATION_SIZE

	#determine deviation in heading/desired heading
	if i<DISCRETIZATION_SIZE-1:
		h_dev=np.arctan2(velocity[0]*pHeadings[1,i]-velocity[1]*pHeadings[0,i],velocity[0]*pHeadings[0,i]+velocity[1]*pHeadings[1,i])
	else:
		h_dev=0
	#determine deviation from path wrt allowed error
	p_dev=path[:,i]-position[0:2]
	p_dev_th=np.arctan2(p_dev[1],p_dev[0])
	p_dev_n=np.linalg.norm(p_dev)
	#apply control law on path parameter, heading deviation, path deviation
	throttle=(1-t)*(-1)**backwards

	heading=position[2]
	if backwards:
		if heading>0:
			heading=heading-np.pi
		elif heading<0:
			heading=heading+np.pi
	if p_dev_n>PATH_TOL:
		turn_ratio=(p_dev_th-heading)/np.pi*(-1)**backwards
		throttle=0
		if abs(turn_ratio)<0.1:
			throttle=(-1)**backwards
	elif abs(h_dev)>np.pi/4:
		turn_ratio=h_dev/np.pi*(-1)**backwards
		throttle=0
	else:
		turn_ratio=h_dev/np.pi*(-1)**backwards

	return [throttle, turn_ratio]
