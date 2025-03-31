from vex import *

class RobotConfig:
    def __init__(self, wheelDiameter, gearRatio, drivebaseWidth, peripherals):
        self.wheelDiameter = wheelDiameter
        self.gearRatio = gearRatio
        self.drivebaseWidth = drivebaseWidth
        self.peripherals = peripherals

    def getWheelDiameter(self):
        return self.wheelDiameter
    
    def getGearRatio(self):
        return self.gearRatio
    
    def getDrivebaseWidth(self):
        return self.drivebaseWidth
    
    def getCircumference(self):
        return math.pi * self.wheelDiameter
    
    def getDPI(self):
        return 360.0 / self.getCircumference()
    
    def getFrontRangeFinder(self):
        return self.peripherals[0]
    
    def getRightRangeFinder(self):
        return self.peripherals[1]
    
    def getInertialSensor(self):
        return self.peripherals[2]

    def getLeftLineSensor(self):
        return self.peripherals[3]
    
    def getRightLineSensor(self):
        return self.peripherals[4]
    
    def getBumpSwitch(self):
        return self.peripherals[5]
    
    def getArmMotor(self):
        return self.peripherals[6]