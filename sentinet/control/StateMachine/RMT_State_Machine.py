from sentinet.control.StateMachine.State_Machine_Base import StateMachineBase, ActionStateBase
from sentinet.core.control.ControlClient import ControlClient, pub_params, sub_params, serve_params, req_params
import numpy as np
import math
from sentinet.core.control import ControlClient
from multiprocessing import Process, Pipe
from time import time, sleep

DISCRETIZATION_SIZE=100
ATTRACTOR=[0,0]
PATH_TOL=0.05 #meters
CHECKSUM="CHECKSUM"
# todo: better transition law
#		action states: getter, callback, requester
class RMT_SM(StateMachineBase):
	#mining_zone=[[x_lower,x_upper],[y_lower,y_upper]]
	#dumping_zone=[[x_lower,x_upper],[y_lower,y_upper]]
	def __init__(self,alphabet,state_list,t_max,init_state=None):
		super().__init__(alphabet,state_list,t_max,init_state=init_state)
		self.init_system()
	
	def init_system(self): #start in initial state
		self.curr_state=0
		self.execute_state(state_list[0])
		self.run_SM()
	
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
				and self.state['a'] is False 
				and self.state['v'] is False 
				and self.state['m'] is False 
				and self.state['d'] is False 
				and self.state['f'] is False):
				return 1

		elif self.curr_state == 1: # Transition from mv2mine
			if ((self.state['x'] < self.mining_zone[0][0] or self.state['x'] > self.mining_zone[0][1])
				and (self.state['y'] < self.mining_zone[1][0] or self.state['y'] > self.mining_zone[1][1]) 
				and self.state['a'] is False  
				and self.state['m'] is False 
				and self.state['d'] is False 
				and self.state['f'] is False):
				return 1
			elif ((self.state['x'] > self.mining_zone[0][0] and self.state['x'] < self.mining_zone[0][1])
				and (self.state['y'] > self.mining_zone[1][0] and self.state['y'] < self.mining_zone[1][1])
				and self.state['a'] is False
				and self.state['m'] is False
				and self.state['d'] is False
				and self.state['f'] is False
				and self.state['v'] is False):
				return 2


		elif self.curr_state == 2: # Transition from mine
			if ((self.state['x'] > self.mining_zone[0][0] and self.state['x'] < self.mining_zone[0][1])
				and (self.state['y'] > self.mining_zone[1][0] and self.state['y'] < self.mining_zone[1][1])
				and self.state['a'] is False
				and self.state['m'] is True
				and self.state['d'] is False
				and self.state['f'] is False
				and self.state['v'] is False):
				return 2

			elif ((self.state['x'] > self.mining_zone[0][0] and self.state['x'] < self.mining_zone[0][1])
				and (self.state['y'] > self.mining_zone[1][0] and self.state['y'] < self.mining_zone[1][1])
				and self.state['a'] is False
				and self.state['m'] is False
				and self.state['d'] is False
				and self.state['f'] is True
				and self.state['v'] is False):
				return 3
		elif self.curr_state == 3: # Transition from mv2dump
			if ((self.state['x'] < self.dumping_zone[0][0] or self.state['x'] > self.dumping_zone[0][1])
				and (self.state['y'] < self.dumping_zone[1][1] or self.state['y'] > self.dumping_zone[1][1])
				and self.state['a'] is False
				and self.state['m'] is False
				and self.state['d'] is False
				and self.state['f'] is True):
				return 3
			elif ((self.state['x'] > self.dumping_zone[0][0] and self.state['x'] < self.dumping_zone[0][1])
				and (self.state['y'] > self.dumping_zone[1][0] and self.state['y'] < self.dumping_zone[1][1])
				and self.state['a'] is False
				and self.state['m'] is False
				and self.state['d'] is False
				and self.state['f'] is True
				and self.state['v'] is False):
				return 4

		elif self.curr_state == 4: # Transition from dump
			if ((self.state['x'] > self.dumping_zone[0][0] and self.state['x'] < self.dumping_zone[0][1])
				and (self.state['y'] > self.dumping_zone[1][0] and self.state['y'] < self.dumping_zone[1][1])
				and self.state['a'] is False
				and self.state['m'] is False
				and self.state['d'] is True
				and self.state['f'] is True
				and self.state['v'] is False):
				return 4
			elif ((self.state['x'] > self.dumping_zone[0][0] and self.state['x'] < self.dumping_zone[0][1])
				and (self.state['y'] > self.dumping_zone[1][0] and self.state['y'] < self.dumping_zone[1][1])
				and self.state['a'] is False
				and self.state['m'] is False
				and self.state['d'] is False
				and self.state['f'] is False
				and self.state['v'] is False):
				return 1

	def update_system_state(self): #update sys_state from localizer
		pos=self.localizer_callback()
		self.state['x']=pos[0]
		self.state['y']=pos[1]
		self.state['th']=pos[2]

	def localizer_callback(self): #callback to localizer
		return 0

	def run_SM(self): #master run loop
		while True:
			self.update_system_state()
			pipe_check=self.read_pipe()
			if pipe_check is not None:
				keys=pipe_check.keys()
				if 'fin' in keys:
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
	def __init__(self,pipe):
		super().__init__(pipe)

	def execute(self):
		self.target=self.select_target_zone()
		self.path=self.determine_path()
		self.run_PD()
		self.end_state()

	def build_pub_sub(self):
		self.pub_params=pub_params()
		self.pub_params.get_data=self.serialize_data
		self.pub_params.topic="cmd_vel"
		self.pub_params.period=10
		self.CC.spin(self.pub_params)

	def serialize_data(self): #TODO
		return 0

	def select_target_zone(self): #select target pos from zone as np array, zone boundaries hard coded from reqs
		return 0

	def determine_path(self): #Bezier Curve Path Generator
		self.state=self.get_state()
		self.path,self.dpath=Bez_Cur([self.state['x'],self.state['y']],self.target,ATTRACTOR)

	def run_PD(self): #while loop run of PD controller
		self.pipe_value(dict(moving=True))
		self.np_pos=np.array([self.state['x'],self.state['y']])
		self.vel=np.array([0,0])
		self.pos_diff_norm=np.linalg.norm(self.np_pos-self.target)
		while self.pos_diff_norm>PATH_TOL:
			self.set_data(GLPDC(self.path,self.dpath,self.np_pos,self.vel,0))
			self.state=self.get_state()
			self.vel=np.array([[self.state['x'],self.state['y']]])-self.np_pos
			self.np_pos=np.array([self.state['x'],self.state['y']])
			self.pos_diff_norm=np.linalg.norm(self.np_pos-self.target)
		self.set_data([0,0])
		sleep(0.05)
		self.pipe_value(dict(moving=False))

class mine(ActionStateBase): #mining action state
	def __init__(self,pipe):
		super().__init__(pipe)
		self.done=False

	def callback(self,string): #TODO
		if string == 'done'+CHECKSUM:
			self.pipe_value(dict(mining=False,full=True))
			self.done=True
		return string+CHECKSUM

	def build_pub_sub(self): #TODO
		self.CC.serve(self.callback)

	def execute(self):
		self.mine_flag_handler()

	def serialize_data(self): #TODO
		return 0

	def mine_flag_handler(self): #wait for finished flag from low level
		self.pipe_value(dict(mining=True))
		self.CC.request("tcp://localhost:5555","start_mining")
		while not self.done:
			sleep(0.05)
		self.end_state()

class mv2dump(ActionStateBase): #moving to dumping zone mining state
	def __init__(self,pipe):
		super().__init__(pipe)

	def execute(self):
		self.target=self.select_target_zone()
		self.path=self.determine_path()
		self.run_PD()
		self.end_state()

	def build_pub_sub(self):
		self.pub_params=pub_params()
		self.pub_params.get_data=self.serialize_data
		self.pub_params.topic="cmd_vel"
		self.pub_params.period=10
		self.CC.spin(self.pub_params)

	def serialize_data(self): #TODO
		return 0

	def select_target_zone(self): #hard coded return point based on reqs
		return 0

	def determine_path(self): #Bezier curve path generator
		self.state=self.get_state()
		self.path,self.dpath=Bez_Cur([self.state['x'],self.state['y']],self.target,ATTRACTOR)

	def run_PD(self): #while loop for PD controller
		self.pipe_value(dict(moving=True))
		self.np_pos=np.array([self.state['x'],self.state['y']])
		self.vel=np.array([0,0])
		self.pos_diff_norm=np.linalg.norm(self.np_pos-self.target)
		while self.pos_diff_norm>PATH_TOL:
			self.set_data(GLPDC(self.path,self.dpath,self.np_pos,self.vel,1))
			self.state=self.get_state()
			self.vel=np.array([[self.state['x'],self.state['y']]])-self.np_pos
			self.np_pos=np.array([self.state['x'],self.state['y']])
			self.pos_diff_norm=np.linalg.norm(self.np_pos-self.target)
		self.set_data([0,0])
		sleep(0.05)
		self.pipe_value(dict(moving=False))

class dump(ActionStateBase):
	def __init__(self,pipe):
		super().__init__(pipe)
		self.done=False

	def callback(self,string): #TODO
		if string == 'done'+CHECKSUM:
			self.pipe_value(dict(dumping=False,full=False))
			self.done=True
		return string+CHECKSUM

	def build_pub_sub(self): #TODO
		self.CC.serve(self.callback)

	def execute(self):
		self.dump_flag_handler()

	def serialize_data(self): #TODO
		return 0

	def dump_flag_handler(self): #wait for finished flag from low level
		self.pipe_value(dict(dumping=True))
		self.CC.request("tcp://localhost:5555","start_dumping")
		while not self.done:
			sleep(0.05)
		self.end_state()

class init_state(ActionStateBase): #initialization state
	def __init__(self,pipe):
		super().__init__(pipe)
		self.stowed=True

	def execute(self):
		self.dep_auger()
		self.state=self.get_state()
		self.find_self()
		self.end_state()

	def callback(self):
		if string == 'done'+CHECKSUM:
			self.pipe_value(dict(stowed=false))
			self.stowed=False
		return string+CHECKSUM

	def build_pub_sub(self):
		self.CC.serve(self.callback)

	def serialize_data(self): #TODO
		return 0

	def dep_auger(self): #send auger deployment command to low level
		self.CC.request("tcp://localhost:5555","deploy_auger")
		while self.stowed:
			sleep(0.05)		
	
	def scan_camera(): # TODO spin camera 360
		return 0

	def find_self(self): # if position is unknown scan camera
		while self.state is None:
			self.state=self.get_state()
		if self.state['x'] is None:
			self.scan_camera()

class soft_exit(ActionStateBase): #planned soft exit state
	def __init__(self,pipe):
		super().__init__(pipe)
		self.done=False

	def execute(self):
		self.exit_handler()
		self.pipe_value(dict(fin=True))
		self.end_state()

	def serialize_data(self): #TODO
		return 0

	def callback(self):
		if string == 'done'+CHECKSUM:
			self.done=True
		return string+CHECKSUM

	def build_pub_sub(self):
		self.CC.serve(self.callback)

	def exit_handler(self):
		self.CC.request("tcp://localhost:5555","soft_exit")
		while not self.done:
			sleep(0.05)

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
