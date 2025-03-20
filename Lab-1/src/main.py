# ---------------------------------------------------------------------------- #
#                                                                              #
# 	Module:       main.py                                                      #
# 	Author:       Seth Warren, Logan Bachman                                   #
# 	Created:      3/19/2025, 7:45:29 AM                                        #
# 	Description:  V5 project                                                   #
#                                                                              #
# ---------------------------------------------------------------------------- #

# Library imports
from vex import *

# Brain should be defined by default
brain=Brain()

left_motor = Motor(Ports.PORT10, 18_1, True)
right_motor = Motor(Ports.PORT1, 18_1, False)

left_motor.set_velocity(30, RPM)
left_motor.reset_position()
right_motor.set_velocity(30, RPM)
right_motor.reset_position()

wheelDiameter = 4.0
wheelCircumference = 3.14 * wheelDiameter
gearRatio = 5.0
wheelTrack = 11.0 
degreesPerInch = 360.0 / wheelCircumference

def moveLen(len):
    deg = 360 * ((len / wheelCircumference) * gearRatio)

    left_motor.spin_for(FORWARD, deg, DEGREES, 100, RPM, False)
    right_motor.spin_for(FORWARD, deg, DEGREES, 100, RPM, True)

def turnDeg(deg, pivotOffset = 0.0):
    lenLeft = -(2 * 3.14 * ((wheelTrack / 2) + pivotOffset)) * (-deg / 360)
    lenRight = (2 * 3.14 * ((wheelTrack / 2) - pivotOffset) * (-deg / 360))

    degLeft = 360 * ((lenLeft / wheelCircumference) * gearRatio)
    degRight = 360 * ((lenRight / wheelCircumference) * gearRatio)

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

drivePolygon(4, 8)



# ==========   EXTRAS   ==========

#x y theta
#x is forward
#y is left right
odom = [0,0,0]
brain.screen.clear_screen()
brain.screen.print_at("X: " + str(odom[0]), x=0, y=0)
brain.screen.print_at("Y: " + str(odom[1]), x=0, y=20)
brain.screen.print_at("Heading: " + str(odom[2]), x=0, y=40)



def runOneMotor(motor, distance, unit, wait):
    motor.spin_for(FORWARD, distance*gearRatio, unit, wait)

def driveDistance(distanceInInches, gearRatio=5):
    left_motor.spin_for(FORWARD, distanceInInches*degreesPerInch*gearRatio, DEGREES, 100, RPM, False)
    right_motor.spin_for(FORWARD, distanceInInches*degreesPerInch*gearRatio, degreesPerInch, 100, RPM, True)
    odom[0] += distanceInInches * math.cos(math.radians(odom[2]))
    odom[1] += distanceInInches * math.sin(math.radians(odom[2]))
    brain.screen.clear_screen()
    brain.screen.print_at("X: " + str(odom[0]), x=0, y=0)
    brain.screen.print_at("Y: " + str(odom[1]), x=0, y=20)

def turnDegrees(robotTurnInDegrees, gearRatio=5):
    turnDistance = wheelTrack / wheelDiameter
    left_motor.spin_for(REVERSE, turnDistance*robotTurnInDegrees*degreesPerInch*gearRatio, DEGREES, 100, RPM, False)
    right_motor.spin_for(FORWARD, turnDistance*robotTurnInDegrees*degreesPerInch*gearRatio, DEGREES, 100, RPM, True)
    odom[2] += robotTurnInDegrees
    brain.screen.clear_screen()
    brain.screen.print_at("Heading: " + str(odom[2]), x=0, y=40)


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

def LeftRightBoth():
    rotations = 1
    runOneMotor(left_motor, rotations, TURNS, True)
    wait(1000)
    runOneMotor(right_motor, rotations, TURNS, True)
    wait(1000)
    
    # should be 5 turns because of the 60/12 gearing to the wheels
    runOneMotor(right_motor, rotations, TURNS, False)
    runOneMotor(left_motor, rotations, TURNS, True)
