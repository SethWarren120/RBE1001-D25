from vex import *
from constants import *
from Subsystems.subsystem import *

class Forks(Subsystem):
    def __init__(self, forkmotor: Motor):
        self.forkmotor = forkmotor

        self.forksDeployed = False
        self.basketContains = []
        self.nextSlot = 0,0
        self.currentSlot = 0

    def deployForks(self):
        self.forkmotor.spin_for(FORWARD, 90 * forksGearRatio, DEGREES)
        self.forksDeployed = True

    def retractForks(self):
        self.forkmotor.spin_for(FORWARD, -90 * forksGearRatio, DEGREES)
        self.forksDeployed = False

    def toggleForks(self):
        if self.forksDeployed:
            self.retractForks()
        else:
            self.deployForks()
        self.currentCommand = None

    def addObject(self, row, column):
        if len(self.basketContains) < row-1:
            if (column == 1):
                self.basketContains.append((1, 0))
            else:
                self.basketContains.append((0, 1))
        else:
            if (column == 1):
                self.basketContains[row-1] = (1, 0)
            else:
                self.basketContains[row-1] = (0, 1)
        
        self.nextSlot = self.getLeftestSlot()
    
    def getLeftestSlot(self):
        for i in range(len(self.basketContains)):
            for j in range(len(self.basketContains)):
                if self.basketContains[i][j] == 0:
                    return i+1,j+1
        return -1,-1
    
    def retractForksCommand(self):
        self.run(self.retractForks())
        self.currentCommand = None
        
    def toggleForksCommand(self):
        self.run(self.toggleForks())
        self.currentCommand = None