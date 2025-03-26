# Library imports
from vex import *
#from util import *
from Subsystems.Drivebase.drivebase import Drivebase
from Subsystems.Drivebase.drivebaseMotorCorrector import *

class TankDrivebase (Drivebase):

    diameter = 4
    gearing = 5 / 1
    wheelBase = 11
    circumference = math.pi * diameter

    def __init__(self, _motorLeft, _motorRight, rangeFinderRight, rangeFinderFront, wheelDiameter, gearRatio, drivebaseWidth,
                 motorCorrectionConfig = None, _rotationUnits = DEGREES, _speedUnits = RPM, kP = 0.5):
        self.motorLeft = _motorLeft
        self.motorRight = _motorRight

        self.diameter = wheelDiameter
        self.gearing = gearRatio
        self.wheelBase = drivebaseWidth
        self.circumference = math.pi * wheelDiameter
        self.dpi = 360.0 / self.circumference

        self.rotationUnits = _rotationUnits
        self.speedUnits = _speedUnits
        self.kP = kP
        self.rangeFinderRightSide = rangeFinderRight
        self.rangeFinderFront = rangeFinderFront

        self.desiredDistance = 8.0

        if motorCorrectionConfig == None:
            motorCorrectionConfig = DrivebaseMotorCorrectionProfile.Disabled(_rotationUnits)
        else:
            motorCorrectionConfig.units = _rotationUnits

        self.motorCorrector = DrivebaseMotorCorrector([_motorLeft, _motorRight], motorCorrectionConfig)

    def moveLen(self, len, speed):
        deg = 360 * ((len / self.circumference) * self.gearing)

        self.motorCorrector.setPassiveMode(False)
        self.motorCorrector.correctMotors([deg, deg])

        self.motorLeft.spin_for(FORWARD, deg, self.rotationUnits, speed, self.speedUnits, False)
        self.motorRight.spin_for(FORWARD, deg, self.rotationUnits, speed, self.speedUnits, True)

    def turnDeg(self, rotation, speed, pivotOffset = 0.0):
        lenLeft = -(2 * math.pi * ((self.wheelBase / 2) + pivotOffset)) * (-rotation / 360)
        lenRight = (2 * math.pi * ((self.wheelBase / 2) - pivotOffset)) * (-rotation / 360)

        degLeft = 360 * ((lenLeft / self.circumference) * self.gearing)
        degRight = 360 * ((lenRight / self.circumference) * self.gearing)

        self.motorCorrector.setPassiveMode(False)
        self.motorCorrector.correctMotors([degLeft, degRight])

        if (degLeft > degRight):
            self.motorLeft.spin_for(FORWARD, degLeft, DEGREES, speed, self.speedUnits, False)
            self.motorRight.spin_for(FORWARD, degRight, DEGREES, speed * (degRight / degLeft), self.speedUnits, True)
        else:
            self.motorLeft.spin_for(FORWARD, degLeft, DEGREES, speed * (degLeft / degRight), self.speedUnits, False)
            self.motorRight.spin_for(FORWARD, degRight, DEGREES, speed, self.speedUnits, True)

    
    def drive(self, speed, direction):
        if (direction < 0):
            self.motorLeft.set_velocity(speed+abs(direction), RPM)
            self.motorRight.set_velocity(speed-abs(direction), RPM)
        elif (direction > 0):
            self.motorLeft.set_velocity(speed-abs(direction), RPM)
            self.motorRight.set_velocity(speed+abs(direction), RPM)
        else:
            self.motorLeft.set_velocity(speed, RPM)
            self.motorRight.set_velocity(speed, RPM)
            
        self.motorLeft.spin(FORWARD)
        self.motorRight.spin(FORWARD)

    def wallFollowInches(self, setDistanceFromWall):
        while True:
            rightError = self.rangeFinderRightSide.distance(INCHES) - setDistanceFromWall
            self.drive(100, -self.kP*rightError)

    def driveLab21(self):
        wait(2000)
        self.desiredDistance = 8.0
        self.wallFollowInches(8.0)
        # self.drive(30,30)
        # self.desiredDistance = 8.0
        # self.wallFollowInches(8.0)
        # self.drive(30,30)
