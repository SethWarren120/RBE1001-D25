from vex import *
from constants import *

class Arm ():
    def __init__(self, armmotor, pivotmotor):
        self.armmotor = armmotor
        self.pivotmotor = pivotmotor

        self.armGearRatio = armGearRatio
        self.pivotGearRatio = pivotGearRatio

        self.armHeight = 0
        self.armAngle = 0

    def getHeight(self):
        return self.armHeight

    def setHeight(self, height):
        actualHeight = self.clamp(height, minArmLength, maxArmLength)
        heightDifference = actualHeight - self.armHeight

        self.armmotor.spin_to_position(FORWARD, heightDifference * self.armGearRatio, DEGREES)
        self.armHeight = self.armmotor.position(DEGREES) / self.armGearRatio

    def setAngle(self, angle):
        actualAngle = self.clamp(angle, minArmAngle, maxArmAngle)
        angleDifference = actualAngle - self.armAngle

        self.pivotmotor.spin_to_position(FORWARD, angleDifference * self.pivotGearRatio, DEGREES)
        self.armAngle = self.pivotmotor.position(DEGREES) / self.pivotGearRatio
    
    def getAngle(self):
        return self.armAngle
    
    def clamp(self, value, min_value, max_value):
        return max(min_value, min(value, max_value))