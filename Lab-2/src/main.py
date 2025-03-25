# ---------------------------------------------------------------------------- #
#                                                                              #
# 	Module:       main.py                                                      #
# 	Author:       Seth Warren, Logan Bachman                                                        #
# 	Created:      3/19/2025, 7:45:29 AM                                        #
# 	Description:  V5 project                                                   #
#                                                                              #
# ---------------------------------------------------------------------------- #

# Library imports
from vex import *

#Constants
wheelDiameter = 4.0
wheel_travel = math.pi*wheelDiameter
track_width = 11
wheel_base = 11
gear_ratio = 5
wheelCircumference = 3.14 * wheelDiameter 
degreesPerInch = 360.0 / wheelCircumference
kP = 0.5

brain=Brain()

left_motor = Motor(Ports.PORT10, GearSetting.RATIO_18_1, False)
left_motor.set_velocity(30, RPM)
left_motor.reset_position()
right_motor = Motor(Ports.PORT1, GearSetting.RATIO_18_1, False)
right_motor.set_velocity(30, RPM)
right_motor.reset_position()

rangeFinderFront = Sonar(Ports.PORT4)
rangeFinderRightSide = Sonar(Ports.PORT3)

def moveLen(len):
    deg = 360 * ((len / wheelCircumference) * gear_ratio)

    left_motor.spin_for(FORWARD, deg, DEGREES, 100, RPM, False)
    right_motor.spin_for(FORWARD, deg, DEGREES, 100, RPM, True)

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

def turnDeg(deg, pivotOffset = 0.0):
    lenLeft = -(2 * 3.14 * ((track_width / 2) + pivotOffset)) * (-deg / 360)
    lenRight = (2 * 3.14 * ((track_width / 2) - pivotOffset)) * (-deg / 360)

    degLeft = 360 * ((lenLeft / wheelCircumference) * gear_ratio)
    degRight = 360 * ((lenRight / wheelCircumference) * gear_ratio)

    if (degLeft > degRight):
        left_motor.spin_for(FORWARD, degLeft, DEGREES, 100, RPM, False)
        right_motor.spin_for(FORWARD, degRight, DEGREES, 100 * (degRight / degLeft), RPM, True)
    else:
        left_motor.spin_for(FORWARD, degLeft, DEGREES, 100 * (degLeft / degRight), RPM, False)
        right_motor.spin_for(FORWARD, degRight, DEGREES, 100, RPM, True)

def driveToPose(x,y,theta):
    pass
    # currentPose = odom
    # deltaX = x - currentPose[0]
    # deltaY = y - currentPose[1]
    # deltaTheta = theta - currentPose[2]

    # distance = math.sqrt(deltaX**2 + deltaY**2)
    # angle = math.degrees(math.atan2(deltaY, deltaX))
    
    # turnDegrees(angle - currentPose[2], 5)
    # driveDistance(distance, 5)
    # turnDegrees(theta - angle, 5)

wallFollowInches(11)