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
    
    def __init__(self, _motorLeft, _motorRight, wheelDiameter, gearRatio, drivebaseWidth,
                 motorCorrectionConfig = None, _rotationUnits = DEGREES, _speedUnits = RPM):
        self.motorLeft = _motorLeft
        self.motorRight = _motorRight

        self.diameter = wheelDiameter
        self.gearing = gearRatio
        self.wheelBase = drivebaseWidth
        self.circumference = math.pi * wheelDiameter
        self.dpi = 360.0 / self.circumference

        self.rotationUnits = _rotationUnits
        self.speedUnits = _speedUnits

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

    
def drive(speed, direction):
    motor_direction = FORWARD if speed >= 0 else REVERSE
    speed_abs = abs(speed)
    direction_normalized = max(min(direction / 100.0, 1.0), -1.0)
    left_speed = speed_abs
    right_speed = speed_abs
    
    if direction_normalized > 0:  # Turn left
        left_speed = speed_abs * (1.0 - direction_normalized)
    elif direction_normalized < 0:  # Turn right
        right_speed = speed_abs * (1.0 + direction_normalized)

    left_motor.spin(motor_direction, left_speed, RPM, False)
    right_motor.spin(motor_direction, right_speed, RPM, True)

def wallFollowInches(setDistanceFromWall):
    actualSetpoint = setDistanceFromWall - 1.0
    minDistance = 1.0  # Minimum safe distance (inches)
    maxDistance = 20.0  # Maximum reliable sensor distance (inches)
    
    try:
        while True:
            # Get current distance from wall
            currentDistance = rangeFinderRightSide.distance(INCHES)
            
            error = currentDistance - actualSetpoint
            
            if currentDistance < minDistance:
                steeringCorrection = 75  # Turn left (away from wall)
            elif currentDistance > maxDistance:
                steeringCorrection = -50  # Turn right (toward wall)
            else:
                steeringCorrection = kP * error
                steeringCorrection = max(min(steeringCorrection, 100), -100)
            
            drive(100, steeringCorrection)

            #If it is <8 inches from the front wall, turn left
            if (rangeFinderFront.distance(INCHES) < 8):
                drive(30,100)
    finally:
        # Stop motors when done
        left_motor.stop()
        right_motor.stop()
