from sentinet.control.StateMachine import RMT_State_Machine
from sentinet.control.Localizer.CommsTestLocalizer import DummyLocalizer
import numpy as np

if __name__=='__main__':
	DL = DummyLocalizer(None, None)
	print(None in DL.position)