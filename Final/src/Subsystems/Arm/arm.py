from vex import *
from constants import *

class Arm ():
    def __init__(self, armmotor, pivotmotor):
        self.armmotor = armmotor
        self.pivotmotor = pivotmotor

        self.armGearRatio = armGearRatio
        self.pivotGearRatio = pivotGearRatio

        self.armLength = 0
        self.armAngle = 0

    def getLength(self):
        return self.armLength

    def setLength(self, length):
        actualLength = self.clamp(length, minArmLength, maxArmLength)
        lengthDifference = actualLength - self.armLength

        self.armmotor.spin_to_position(FORWARD, lengthDifference * self.armGearRatio, DEGREES)
        self.armLength = self.armmotor.position(DEGREES) / self.armGearRatio

    def setAngle(self, angle):
        actualAngle = self.clamp(angle, minArmAngle, maxArmAngle)
        angleDifference = actualAngle - self.armAngle

        self.pivotmotor.spin_to_position(FORWARD, angleDifference * self.pivotGearRatio, DEGREES)
        self.armAngle = self.pivotmotor.position(DEGREES) / self.pivotGearRatio
    
    def getAngle(self):
        return self.armAngle
    
    def clamp(self, value, min_value, max_value):
        return max(min_value, min(value, max_value))
    
    def toPosition(self, length, angle):
        self.setLength(length)
        self.setAngle(angle)