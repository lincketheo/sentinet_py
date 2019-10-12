from abc import ABC, abstractmethod

class AbstractLocalizer(ABC):

    @abstractmethod
    def position(self):
        pass

    @abstractmethod
    def velocity(self):
        pass