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

left_motor = Motor(Ports.PORT10, GearSetting.RATIO_18_1, False)
left_motor.set_velocity(30, RPM)
left_motor.reset_position()
right_motor = Motor(Ports.PORT1, GearSetting.RATIO_18_1, False)
right_motor.set_velocity(30, RPM)
right_motor.reset_position()

def updateOdometry():
    while True:
        left_motor.reset_position()
        right_motor.reset_position()

        wait(1500, MSEC)

        left_position = left_motor.position(DEGREES)
        right_position = right_motor.position(DEGREES)
        brain.screen.print_at("left: " + str(left_position), x=0, y=80)
        brain.screen.print_at("right: " + str(right_position), x=0, y=100)

        left_distance = (left_position / 360.0) * wheelCircumference / gear_ratio
        right_distance = (right_position / 360.0) * wheelCircumference / gear_ratio

        avg_distance = (left_distance + right_distance) / 2.0
        delta_theta = (right_distance - left_distance) / track_width
        
        brain.screen.print_at("avg_distance: " + str(avg_distance), x=0, y=120)
        brain.screen.print_at("delta_theta: " + str(delta_theta), x=0, y=140)

        odom[0] += avg_distance * math.cos(math.radians(odom[2]))
        odom[1] += avg_distance * math.sin(math.radians(odom[2]))
        odom[2] += math.degrees(delta_theta)
        if (odom[2] > 360):
            odom[2] -= 360
        elif (odom[2] < 0):
            odom[2] += 360

        brain.screen.print_at("X: " + str(odom[0]), x=0, y=20)
        brain.screen.print_at("Y: " + str(odom[1]), x=0, y=40)
        brain.screen.print_at("Heading: " + str(odom[2]), x=0, y=60)
        brain.screen.render()

#x y theta
#when theta is 0: +x is forward, +y is left
odom = [0.0,0.0,0.0]
odomThread = Thread(updateOdometry)

def moveLen(len):
    deg = 360 * ((len / wheelCircumference) * gear_ratio)

    left_motor.spin_for(FORWARD, deg, DEGREES, 100, RPM, False)
    right_motor.spin_for(FORWARD, deg, DEGREES, 100, RPM, True)

    
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

drivePolygon(4,1,0)