from vex import *
from constants import *

class Forks ():
    def __init__(self, forkmotor, rollerMotor):
        self.forkmotor = forkmotor
        self.rollerMotor = rollerMotor
        self.gearRatio = forksGearRatio
        self.rollerDiameter = rollerDiameter
        self.rollerCircumference = math.pi * rollerDiameter

        self.forksDeployed = False
        self.basketContains = []
        self.nextSlot = 0,0
        self.currentSlot = 0

    def deployForks(self):
        self.forkmotor.spin_for(FORWARD, 90*self.gearRatio, DEGREES)
        self.forksDeployed = True

    def retractForks(self):
        self.forkmotor.spin_for(FORWARD, -90*self.gearRatio, DEGREES)
        self.forksDeployed = False

    def rollBasket(self, distance):
        self.rollerMotor.spin_for(FORWARD, distance, DEGREES, True)

    def toggleForks(self):
        if self.forksDeployed:
            self.retractForks()
        else:
            self.deployForks()
    
    def moveToFirstSlot(self):
        #definitely doesn't work
        if self.forksDeployed and self.nextSlot[0] != -1:
            self.nextSlot = self.getLeftestSlot()
            distanceToOpenSlotSmall = self.currentSlot * smallFruitWidth
            distanceToOpenSlotLarge = self.currentSlot * largeFruitWidth

            slotDifference = self.nextSlot[0] - self.currentSlot
            smallDriveDegrees = distanceToOpenSlotSmall * slotDifference * self.rollerCircumference * self.gearRatio
            largeDriveDegrees = distanceToOpenSlotLarge * slotDifference * self.rollerCircumference * self.gearRatio

            self.rollBasket(smallDriveDegrees)
            # self.rollBasket(largeDriveDegrees)
            self.addObject(self.nextSlot[0], self.nextSlot[1])

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