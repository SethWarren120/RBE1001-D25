from vex import *
from constants import *
from Subsystems.subsystem import *

class Arm (Subsystem):
    def __init__(self, armmotor, pivotmotor, wristmotor):
        self.armmotor = armmotor
        self.pivotmotor = pivotmotor
        self.wristmotor = wristmotor

        self.armLength = 0
        self.armAngle = 0
        self.wristAngle = 0

    def getLength(self):
        return self.armLength

    def setLength(self, length):
        actualLength = self.clamp(length, minArmLength, maxArmLength)
        lengthDifference = actualLength - self.armLength

        self.armmotor.spin_to_position(FORWARD, lengthDifference * armGearRatio, DEGREES)
        self.armLength = self.armmotor.position(DEGREES) / armGearRatio

    def setAngle(self, angle):
        actualAngle = self.clamp(angle, minArmAngle, maxArmAngle)
        angleDifference = actualAngle - self.armAngle

        self.pivotmotor.spin_to_position(FORWARD, angleDifference * pivotGearRatio, DEGREES)
        self.armAngle = self.pivotmotor.position(DEGREES) / pivotGearRatio
    
    def getAngle(self):
        return self.armAngle
    
    def getWristAngle(self):
        return self.wristmotor.position(DEGREES) / wristGearRatio
    
    def setWristAngle(self, angle):
        actualAngle = self.clamp(angle, minWristAngle, maxWristAngle)
        angleDifference = actualAngle - self.getWristAngle()

        self.wristmotor.spin_to_position(FORWARD, angleDifference * wristGearRatio, DEGREES)
        self.wristAngle = self.wristmotor.position(DEGREES) / wristGearRatio

    def toPositionCommand(self, length, angle, wristAngle):
        self.setLength(length)
        self.setAngle(angle)
        self.setWristAngle(wristAngle)
        self.currentCommand = None

    def clamp(self, value, min_value, max_value):
        return max(min_value, min(value, max_value))