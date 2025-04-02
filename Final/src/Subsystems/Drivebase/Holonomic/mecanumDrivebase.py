# Library imports
from vex import *
#from util import *
from Subsystems.Drivebase.drivebase import Drivebase
from Subsystems.Drivebase.drivebaseMotorCorrector import *

class MecanumDrivebase ():

    diameter = 4
    gearing = 5 / 1
    wheelBase = 11
    circumference = math.pi * diameter
    
    def __init__(self, _motorFrontLeft, _motorFrontRight, _motorBackLeft, _motorBackRight, _gyro, wheelDiameter, gearRatio, drivebaseWidth,
                 motorCorrectionConfig = None, _rotationUnits = DEGREES, _speedUnits = RPM):
        self.motorFrontLeft = _motorFrontLeft
        self.motorFrontRight = _motorFrontRight
        self.motorBackLeft = _motorBackLeft
        self.motorBackRight = _motorBackRight
        self.gyro = _gyro

        self.diameter = wheelDiameter
        self.gearing = gearRatio
        self.wheelBase = drivebaseWidth
        self.circumference = math.pi * wheelDiameter
        self.dpi = 360.0 / self.circumference

        self.rotationUnits = _rotationUnits
        self.speedUnits = _speedUnits

        self.driveDuration = -1

        if motorCorrectionConfig == None:
            motorCorrectionConfig = DrivebaseMotorCorrectionProfile.Disabled(_rotationUnits)
        else:
            motorCorrectionConfig.units = _rotationUnits

        # self.motorCorrector = DrivebaseMotorCorrector([_motorLeft, _motorRight], motorCorrectionConfig)

    import math  # Import math for trigonometric functions

def drive(self, xVel, yVel, rotVel, durationSeconds=-1):
    startTime = time.time()  # Record the start time

    while durationSeconds == -1 or (time.time() - startTime < durationSeconds):
        # Get the current heading from the gyro
        heading = self.gyro.heading(DEGREES)
        headingRadians = math.radians(heading)  # Convert heading to radians

        # Apply field-centric transformation
        tempXVel = xVel * math.cos(headingRadians) - yVel * math.sin(headingRadians)
        tempYVel = xVel * math.sin(headingRadians) + yVel * math.cos(headingRadians)

        # Drive the motors with the transformed velocities
        self.motorFrontLeft.spin(FORWARD, tempYVel + tempXVel + rotVel)
        self.motorFrontRight.spin(FORWARD, tempYVel - tempXVel - rotVel)
        self.motorBackLeft.spin(FORWARD, tempYVel - tempXVel + rotVel)
        self.motorBackRight.spin(FORWARD, tempYVel + tempXVel - rotVel)

    # Stop the motors after the loop ends
    self.motorFrontLeft.stop()
    self.motorFrontRight.stop()
    self.motorBackLeft.stop()
    self.motorBackRight.stop()