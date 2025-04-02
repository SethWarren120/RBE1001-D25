# Library imports
from abc import ABC, abstractmethod
from vex import *

class Odometry (ABC):

    @abstractmethod
    def updatePosition(*args):
        pass

    @abstractmethod
    def getPosition():
        pass