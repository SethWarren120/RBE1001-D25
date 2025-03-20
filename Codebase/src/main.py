# ---------------------------------------------------------------------------- #
#                                                                              #
# 	Module:       main.py                                                      #
# 	Author:       Logan Bachman                                                        #
# 	Created:      3/19/2025, 7:45:29 AM                                        #
# 	Description:  V5 project                                                   #
#                                                                              #
# ---------------------------------------------------------------------------- #

# Library imports
from vex import *

# Brain should be defined by default
brain=Brain()

brain.screen.print("Hello V5")

left_motor = Motor(Ports.PORT10, GearSetting.RATIO_18_1, False)
left_motor.set_velocity(30, RPM)
left_motor.reset_position()
right_motor = Motor(Ports.PORT1, GearSetting.RATIO_18_1, False)
right_motor.set_velocity(30, RPM)
right_motor.reset_position()

wheelDiameter = 4.0
wheel_travel = math.pi*wheelDiameter
track_width = 11
wheel_base = 11
gear_ratio = 5
wheelCircumference = 3.14 * wheelDiameter 
degreesPerInch = 360.0 / wheelCircumference

#x y theta
#x is forward
#y is left right
odom = [0,0,0]
brain.screen.clear_screen()
brain.screen.print_at("X: " + str(odom[0]), x=0, y=0)
brain.screen.print_at("Y: " + str(odom[1]), x=0, y=20)
brain.screen.print_at("Heading: " + str(odom[2]), x=0, y=40)

def runOneMotor(motor, distance, unit, wait):
    motor.spin_for(FORWARD, distance*gear_ratio, unit, wait)

def driveDistance(distanceInInches, gearRatio=5):
    left_motor.spin_for(FORWARD, distanceInInches*degreesPerInch*gearRatio, DEGREES, 100, RPM, False)
    right_motor.spin_for(FORWARD, distanceInInches*degreesPerInch*gearRatio, degreesPerInch, 100, RPM, True)
    odom[0] += distanceInInches * math.cos(math.radians(odom[2]))
    odom[1] += distanceInInches * math.sin(math.radians(odom[2]))
    brain.screen.clear_screen()
    brain.screen.print_at("X: " + str(odom[0]), x=0, y=0)
    brain.screen.print_at("Y: " + str(odom[1]), x=0, y=20)

def turnDegrees(robotTurnInDegrees, gearRatio=5):
    turnDistance = track_width / wheelDiameter
    left_motor.spin_for(REVERSE, turnDistance*robotTurnInDegrees*degreesPerInch*gearRatio, DEGREES, 100, RPM, False)
    right_motor.spin_for(FORWARD, turnDistance*robotTurnInDegrees*degreesPerInch*gearRatio, DEGREES, 100, RPM, True)
    odom[2] += robotTurnInDegrees
    brain.screen.clear_screen()
    brain.screen.print_at("Heading: " + str(odom[2]), x=0, y=40)

def drivePolygon(numSide, sideLength):
    for i in range(numSide):
        driveDistance(sideLength, 5)
        turnDegrees(360/numSide,5)

def driveMaze():
    driveDistance(29.6, 5)
    turnDegrees(90, 5)
    driveDistance(39, 5)
    turnDegrees(-90, 5)
    driveDistance(29.6, 5)
    turnDegrees(-90, 5)
    driveDistance(7, 5)

#I got bored
def driveToPose(x,y,theta):
    currentPose = odom
    deltaX = x - currentPose[0]
    deltaY = y - currentPose[1]
    deltaTheta = theta - currentPose[2]

    distance = math.sqrt(deltaX**2 + deltaY**2)
    angle = math.degrees(math.atan2(deltaY, deltaX))
    
    turnDegrees(angle - currentPose[2], 5)
    driveDistance(distance, 5)
    turnDegrees(theta - angle, 5)

# def LeftRightBoth():
#     rotations = 1
#     runOneMotor(left_motor, rotations, TURNS, True)
#     wait(1000)
#     runOneMotor(right_motor, rotations, TURNS, True)
#     wait(1000)
    
#     # should be 5 turns because of the 60/12 gearing to the wheels
#     runOneMotor(right_motor, rotations, TURNS, False)
#     runOneMotor(left_motor, rotations, TURNS, True)

# LeftRightBoth()