
from abc, import ABC, abstractmethod

class AbstractLocalizer(ABC)
    def __init__(self,value):
        self.value = value
        super().__init__()

        @abstractmethod
        def position(self):
            pass

        @abstractmethod
        def velocity(self):
            pass

        @abstractmethod
        def acceleration(self):
            pass

