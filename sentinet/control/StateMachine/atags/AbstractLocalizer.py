from abc, import ABC, abstractmethod
from multiprocessing import Pipe
from sentinet.control.Localizer.Localizer_Base import LocalizerBase, SensorBase
from threading import Lock
from sentinet.curmt import KermitControlModule
import numpy as np

class AprilTagsLocalizer(LocalizerBase)

    def __init__(self,pipe,sensors):
        super().__init__(pipe,sensors):
            if None in self.position:
			    self.position.fill(0.0)
		    if None in self.velocity:
			    self.velocity.fill(0.0)
		    if None in self.ang_position:
			    self.ang_position.fill(0.0)
            if None in self.ang_velocity:
                self.ang_velocity.fill(0.0)
 
        @abstractmethod
        def position(self):
            

        @abstractmethod
        def velocity(self):
            pass

        @abstractmethod
        def acceleration(self):
            pass

class IMUSensorBase(SensorBase):
    float angularAccel
    float linearAccel
    float posX
    float posY
    float pozZ
    
    def __init__(self):
        super().__init__(pipe,sensor):
            angularAccel. 
            linearAccel. 
            posX. 
            posY.
            pozZ.
        
        
