import State_Machine_Base
import numpy as np
import math
from multiprocessing import Process, Pipe
from time import time

DISCRETIZATION_SIZE=100
ATTRACTOR=[0,0]
PATH_TOL=0.05 #meters

class RMT_SM(StateMachineBase):
	def __init__(self,alphabet,transition_law,state_list,t_max,init_state=None):
		super().__init__(alphabet,transition_law,state_list,t_max,init_state=init_state)
		self.init_system()
	
	def init_system(self): #start in initial state
		self.curr_state=0
		self.execute_state(state_list[0])
		self.run_SM()
	
	def transistion_law(self): #transition law defined in RMT SM Definition, see Drive
		return 0

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

	def select_target_zone(self): #select target pos from zone as np array, zone boundaries hard coded from reqs
		return 0

	def determine_path(self): #Bezier Curve Path Generator
		self.state=self.get_state()
		self.path,self.dpath=Bez_Cur([self.state['x'],self.state['y']],self.target,ATTRACTOR)

	def run_PD(self): #while loop run of PD controller
		self.pipe_value(dict('moving'=True))
		self.np_pos=np.array([self.state['x'],self.state['y']])
		self.vel=np.array([0,0])
		self.pos_diff_norm=np.linalg.norm(self.np_pos-self.target)
		while self.pos_diff_norm>PATH_TOL:
			self.send_mv_comm(GLPDC(self.path,self.dpath,self.np_pos,self.vel,0))
			self.state=self.get_state()
			self.vel=np.array([[self.state['x'],self.state['y']]])-self.np_pos
			self.np_pos=np.array([self.state['x'],self.state['y']])
			self.pos_diff_norm=np.linalg.norm(self.np_pos-self.target)
		self.pipe_value(dict('moving'=False))

	def send_mv_comm(self,lin_ang): #send lin_ang commands to low level
		return 0


class mine(ActionStateBase): #mining action state
	def __init__(self,pipe):
		super().__init__(pipe)

	def execute(self):
		self.send_mine_comm()
		self.wait4fin()
		self.end_state()

	def wait4fin(self): #wait for finished flag from low level
		return 0

	def send_mine_comm(self): #send mining flag to low level
		self.pipe_value(dict('mining'=True))

		self.pipe_value(dict('mining'=False,'full'=True))
		return 0

class mv2dump(ActionStateBase): #moving to dumping zone mining state
	def __init__(self,pipe):
		super().__init__(pipe)

	def execute(self):
		self.target=self.select_target_zone()
		self.path=self.determine_path()
		self.run_PD()
		self.end_state()

	def select_target_zone(self): #hard conded return point based on reqs
		return 0

	def determine_path(self): #Bezier curve path generator
		self.state=self.get_state()
		self.path,self.dpath=Bez_Cur([self.state['x'],self.state['y']],self.target,ATTRACTOR)

	def run_PD(self): #while loop for PD controller
		self.pipe_value(dict('moving'=True))
		self.np_pos=np.array([self.state['x'],self.state['y']])
		self.vel=np.array([0,0])
		self.pos_diff_norm=np.linalg.norm(self.np_pos-self.target)
		while self.pos_diff_norm>PATH_TOL:
			self.send_mv_comm(GLPDC(self.path,self.dpath,self.np_pos,self.vel,1))
			self.state=self.get_state()
			self.vel=np.array([[self.state['x'],self.state['y']]])-self.np_pos
			self.np_pos=np.array([self.state['x'],self.state['y']])
			self.pos_diff_norm=np.linalg.norm(self.np_pos-self.target)
		self.pipe_value(dict('moving'=False))

	def send_mv_comm(self,lin_ang): #send lin_ang command to low level
		return 0

class dump(ActionStateBase):
	def __init__(self,pipe):
		super().__init__(pipe)

	def execute(self):
		self.send_dump_comm()
		self.wait4fin()
		self.end_state()

	def wait4fin(self): #wait for finished flag from low level
		return 0

	def send_dump_comm(self): #send dumping command to low level
		self.pipe_value(dict('dumping'=True))

		self.pipe_value(dict('dumping'=False,'full'=False))
		return 0

class init_state(ActionStateBase): #initialization state
	def __init__(self,pipe):
		super().__init__(pipe)

	def execute(self):
		self.dep_auger()
		self.find_self()
		self.end_state()

	def dep_auger(self): #send auger deployment command to low level
		self.pipe_value(dict('stowed'=False))		
		return 0
	
	def scan_camera(): #scan camera across stepper limits
		return 0

	def spin_pi(): #turn bot pi rad
		return 0

	def find_self(self): # if position is unknown scan camera, if still rotate bot pi rad and scan again
		if self.state['x']==None:
			self.scan_camera()
			if self.state['x']==None:
				self.spin_pi()
				self.scan_camera()

class soft_exit(ActionStateBase): #planned soft exit state
	def __init__(self,pipe):
		super().__init__(pipe)

	def execute(self):
		self.stop_motors()
		self.send_dump_comm()
		self.stow_auger()
		self.pipe_value(dict('fin'=True))
		self.end_state()

	def stop_motors(self): #send motor kill command to low level
		
		self.pipe_value(dict('moving'=False))
		return 0

	def send_dump_comm(self): #send dumping command to low level
		self.pipe_value(dict('dumping'=True))

		self.pipe_value(dict('dumping'=False,'full'=False))
		return 0

	def stow_auger(self): #send stow auger command to low level
		
		self.pipe_value(dict('stowed'=True))
		return 0

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
