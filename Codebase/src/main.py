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

brain=Brain()
brain.screen.print("Hello V5")

left_motor = Motor(Ports.PORT10, GearSetting.RATIO_18_1, False)
left_motor.set_velocity(30, RPM)
left_motor.reset_position()
right_motor = Motor(Ports.PORT1, GearSetting.RATIO_18_1, False)
right_motor.set_velocity(30, RPM)
right_motor.reset_position()

#x y theta
#when theta is 0: +x is forward, +y is left
odom = [0,0,0]
brain.screen.clear_screen()
brain.screen.print_at("X: " + str(odom[0]), x=0, y=0)
brain.screen.print_at("Y: " + str(odom[1]), x=0, y=20)
brain.screen.print_at("Heading: " + str(odom[2]), x=0, y=40)

def moveLen(len):
    deg = 360 * ((len / wheelCircumference) * gear_ratio)

    left_motor.spin_for(FORWARD, deg, DEGREES, 100, RPM, False)
    right_motor.spin_for(FORWARD, deg, DEGREES, 100, RPM, True)

    odom[0] += len * math.cos(math.radians(odom[2]))
    odom[1] += len * math.sin(math.radians(odom[2]))

    brain.screen.clear_screen()
    brain.screen.print_at("X: " + str(odom[0]), x=0, y=0)
    brain.screen.print_at("Y: " + str(odom[1]), x=0, y=20)
    
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

    arc_length = 2 * math.pi * pivotOffset * (deg / 360)
    delta_x = arc_length * math.cos(math.radians(odom[2]))
    delta_y = arc_length * math.sin(math.radians(odom[2]))
    
    odom[2] += deg
    odom[0] += delta_x
    odom[1] += delta_y

    brain.screen.clear_screen()
    brain.screen.print_at("X: " + str(odom[0]), x=0, y=0)
    brain.screen.print_at("Y: " + str(odom[1]), x=0, y=20)
    brain.screen.print_at("Heading: " + str(odom[2]), x=0, y=40)

def driveMaze():
    moveLen(15)
    turnDeg(-90, -10)
    moveLen(7)
    turnDeg(69, 10)
    turnDeg(21, 20)
    moveLen(2.5)
    turnDeg(90, 3)
    moveLen(5)


def drivePolygon(numSide, sideLength, turnRadius = 7):
    for i in range(numSide):
        moveLen(sideLength)
        turnDeg(360/numSide, turnRadius)

def driveToPose(x,y,theta):
    currentPose = odom
    # deltaX = x - currentPose[0]
    # deltaY = y - currentPose[1]
    # deltaTheta = theta - currentPose[2]

    # distance = math.sqrt(deltaX**2 + deltaY**2)
    # angle = math.degrees(math.atan2(deltaY, deltaX))
    
    # turnDegrees(angle - currentPose[2], 5)
    # driveDistance(distance, 5)
    # turnDegrees(theta - angle, 5)