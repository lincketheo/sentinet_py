from sentinet.control.StateMachine.RMT_State_Machine import RMT_SM, init_state, mv2mine, mine, mv2dump, dump, soft_exit
from sentinet.control.Localizer.CommsTestLocalizer import DummyLocalizer, DummySensor
import numpy as np

if __name__ == '__main__':
	sensor = {'DummySensor': DummySensor()}
	alphabet = ['x', 'y', 'th', 'a', 'm', 'd', 'f', 'v', 's']
	state_list = [init_state, mv2mine, mine, mv2dump, dump, soft_exit]
	tmax = 60
	localizer = DummyLocalizer
	init_state = [0, 0, 0, False, False, False, False, False, True]
	RMT_SM(alphabet, state_list, tmax, localizer, sensor, init_state=init_state)


