# Library imports
from vex import *

from constants import *
from Subsystems.Drivebase.drivebaseMotorCorrector import *
from Subsystems.subsystem import Subsystem

class TankDrivebase (Subsystem):
    def __init__(self, _motorLeft, _motorRight, gyro, vision,
                 motorCorrectionConfig = None, _rotationUnits = DEGREES, _speedUnits = RPM):
        self.motorLeft = _motorLeft
        self.motorRight = _motorRight
        self.gyro = gyro
        self.vision = vision

        self.rotationUnits = _rotationUnits
        self.speedUnits = _speedUnits

        self.x = 0.0
        self.y = 0.0
        self.heading = 0.0

        odometryThread = Thread(self.updateOdometry)
        
        if motorCorrectionConfig == None:
            motorCorrectionConfig = DrivebaseMotorCorrectionProfile.Disabled(_rotationUnits)
        else:
            motorCorrectionConfig.units = _rotationUnits

        self.motorCorrector = DrivebaseMotorCorrector([_motorLeft, _motorRight], motorCorrectionConfig)

    def moveLen(self, len, speed):
        deg = 360 * ((len / wheelCircumference) * gear_ratio)

        self.motorCorrector.setPassiveMode(False)
        self.motorCorrector.correctMotors([deg, deg])

        self.motorLeft.spin_for(FORWARD, deg, self.rotationUnits, speed, self.speedUnits, False)
        self.motorRight.spin_for(FORWARD, deg, self.rotationUnits, speed, self.speedUnits, True)

    def turnDeg(self, rotation, speed, pivotOffset = 0.0):
        lenLeft = -(2 * math.pi * ((wheel_base / 2) + pivotOffset)) * (-rotation / 360)
        lenRight = (2 * math.pi * ((wheel_base / 2) - pivotOffset)) * (-rotation / 360)

        degLeft = 360 * ((lenLeft / wheelCircumference) * gear_ratio)
        degRight = 360 * ((lenRight / wheelCircumference) * gear_ratio)

        self.motorCorrector.setPassiveMode(False)
        self.motorCorrector.correctMotors([degLeft, degRight])

        if (degLeft > degRight):
            self.motorLeft.spin_for(FORWARD, degLeft, DEGREES, speed, self.speedUnits, False)
            self.motorRight.spin_for(FORWARD, degRight, DEGREES, speed * (degRight / degLeft), self.speedUnits, True)
        else:
            self.motorLeft.spin_for(FORWARD, degLeft, DEGREES, speed * (degLeft / degRight), self.speedUnits, False)
            self.motorRight.spin_for(FORWARD, degRight, DEGREES, speed, self.speedUnits, True)

    def updateOdometry(self):
        leftDistance = self.motorLeft.position(DEGREES) / 360.0 * wheelCircumference
        rightDistance = self.motorRight.position(DEGREES) / 360.0 * wheelCircumference

        avgX = (leftDistance - rightDistance) / 4.0
        avgY = (leftDistance + rightDistance) / 4.0
        
        heading = self.gyro.heading(DEGREES)
        headingRadians = math.radians(heading)
        
        deltaX = avgX * math.cos(headingRadians) - avgY * math.sin(headingRadians)
        deltaY = avgX * math.sin(headingRadians) + avgY * math.cos(headingRadians)
        
        self.x += deltaX
        self.y += deltaY
        self.heading = self.gyro.heading(DEGREES)

    def driveCommand(self, leftAxis, rightAxis):
        speed = leftAxis.position()
        turning = rightAxis.position()

        self.motorLeft.spin(FORWARD, speed + turning, self.speedUnits)
        self.motorRight.spin(FORWARD, speed - turning, self.speedUnits)