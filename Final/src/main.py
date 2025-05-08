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

from Subsystems.Drivebase.Tank.tankDrivebase import TankDrivebase
from Subsystems.Arm.arm import Arm
from Subsystems.Intake.intake import Intake
from commands import *
from Control.AutonomousController import *
from Control.Localization import *
from Control.AprilTagProcessor import AprilTagLoop

brain=Brain()

sleep(2000)

motorLeft = Motor(Ports.PORT20, 18_1, False)
motorRight = Motor(Ports.PORT10, 18_1, True)

sideUltrasonic = Sonar(brain.three_wire_port.e)
inertial = Inertial(Ports.PORT1)

arm_motorL = Motor(Ports.PORT17, 18_1, False)
arm_motorR = Motor(Ports.PORT7, 18_1, True)
intake_motor = Motor(Ports.PORT5, 18_1, False)
wrist_motor = Motor(Ports.PORT3, 18_1, False)
pivot_motorL = Motor(Ports.PORT4, 18_1, True)
pivot_motorR = Motor(Ports.PORT9, 18_1, False)

camera = AiVision(Ports.PORT2, vision_Green, vision_Yellow, vision_Orange, vision_Pink, vision_GreenBox, vision_YellowBox, vision_OrangeBox, AiVision.ALL_TAGS)
camera.start_awb()

controller = Controller()

drivebase = TankDrivebase(motorLeft, motorRight, inertial, camera, sideUltrasonic, controller)
intake = Intake(intake_motor)
arm = Arm(arm_motorL, arm_motorR, pivot_motorL, pivot_motorR, wrist_motor)

setSubsystems(drivebase, arm, intake)

def printDebugging():
    while True:
        brain.screen.print_at(arm.getLength(), x=1, y=60)
        brain.screen.print_at(arm.getAngle(), x=1, y=80)
        brain.screen.print_at(arm.getWristAngle(), x=1, y=100)
        sleep(20)

debugThread = Thread(printDebugging)

# ===== AUTO STATE MACHINE =====
#     RAMP = 0
#     TREEDETECTION = 1
#     FRUITDETECTION = 2
#     HARVEST = 3
#     RETREAT = 4
#     SCORE = 5  
#     # Contingency States
#     GYROCORRECTION = 6
#     GROUNDDETECT = 7
#     GROUNDHARVEST = 8 

def grabFruit():
    intake.runIntake(FORWARD)
    arm.setSetpoint(arm.dLength, arm.dAngle, arm.dWrist-5)

# We use threads for out PID loops so that the PIDs always run in the background and do not freeze
armLengthThread = Thread(lambda: arm.setLength())
armAngleThread = Thread(lambda: arm.setAngle())
wristAngleThread = Thread(lambda: arm.setWristAngle())

controller.buttonR2.pressed(grabFruit)
controller.buttonR2.released(intake.stopIntake)

controller.buttonL2.pressed(arm.flipWrist)

controller.buttonB.pressed(dumpObject)

controller.buttonUp.pressed(lambda: arm.setSetpoint(post4Height[0], post4Height[1], post4Height[2]))
controller.buttonDown.pressed(lambda: arm.setSetpoint(post2Height[0], post2Height[1], post2Height[2]))
controller.buttonLeft.pressed(lambda: arm.setSetpoint(post3Height[0], post3Height[1], post3Height[2]))
controller.buttonRight.pressed(lambda: arm.setSetpoint(post1Height[0], post1Height[1], post1Height[2]))

controller.buttonX.pressed(arm.stowArm)

controller.buttonA.pressed(lambda: Ramp())

# The following functions are the ones that correspond to all of the auto states.
# Not all of them have been fully implemented, but have their required components in other parts of the code,
# Likely left unimplemented due to lingering bugs

def Ramp():

    while inertial.orientation(ROLL) > -23:
        drivebase.drive(100, 0)
    
    counter = 1000

    while counter > 0:
        drivebase.drive(40, -(sideUltrasonic.distance(INCHES) - ultrasonicWallClearance) * ultrasonicWallFollowPID[0])
        if inertial.orientation(ROLL) > -5:
            counter -= 1
    pass

    drivebase.drive(0, 0)

# Track which tree and which fruit we are targeting
treeNumber = 2
fruitNumber = -1

# Go to the tree's rough position and figure out its height and rotation using the april tags mounted on the branches.
# The code for the april tag bits exist in AprilTagProcessor.py
def TreeDetection():
    pass

# We know which color the fruit is based on the tree we are targeting
fruitColor = 0
tracking = False

# Find the fruit on the tree and align to it
# This function makes sure that the fruit persists among multiple frames to make sure it is not a false, noisy detection
def FruitDetection():

    # Move the arm up so that the camera is looking up
    arm.setSetpoint(0, 30, 0)
    drivebase.drive(0, 0)

    if treeNumber >= 0 and treeNumber <= 2:
        fruitColor = 0 # Green
    elif treeNumber >= 3 and treeNumber <= 5:
        fruitColor = 2 # Yellow
    else:
        fruitColor = 1 # Orange

    # Timeouts are implemented to make sure the robot does not stall on one step. If it times out, 
    # (timeout = 0 on exit), then the next step for this code would be to return False, to tell the
    # code controlling the state machine that detection failed and to go detect the next fruit instead
    # of trying to harvest a fruit we do not see
    # An exit of a loop with timeout == -1 means that we were successful
    timeout = 75

    continuityCounter = 0

    while timeout > 0:
        if fruitColor == 0:
            objects = camera.take_snapshot(vision_Green)
        elif fruitColor == 1:
            objects = camera.take_snapshot(vision_Orange)
        else:
            objects = camera.take_snapshot(vision_Yellow)
        
        if len(objects) == 0:
            timeout -= 1
            continue

        if 'bestObject' in locals() and tracking:
            compObject = LargestVisionObject(objects)
            print("Yes")
            
            # Checking if the best object we can find roughly aligns with the best one we had last frame, by size and position
            if (abs(compObject.centerX - bestObject.centerX) <= visionContinuityTolerance and 
                    abs(compObject.centerY - bestObject.centerY) <= visionContinuityTolerance and 
                    abs(compObject.width - bestObject.width) <= visionContinuityTolerance and
                    abs(compObject.height - bestObject.height) <= visionContinuityTolerance):
                bestObject = compObject
                continuityCounter += 1
                if continuityCounter == 5:
                    timeout = -1
                    break
            else:
                continuityCounter = 0
                tracking = False
                timeout -= 1
        else:
            bestObject = LargestVisionObject(objects)
            tracking = True

    timeout = 2000

    while (timeout > 0):
        if fruitColor == 0:
            compObject = LargestVisionObject(camera.take_snapshot(vision_Green))
        elif fruitColor == 1:
            compObject = LargestVisionObject(camera.take_snapshot(vision_Orange))
        else:
            compObject = LargestVisionObject(camera.take_snapshot(vision_Yellow))

        # We make sure that if we lose the object, it is possible to timeout and fail, since we do not
        # 100% trust the last step, as color blobs are error prone
        if not (abs(compObject.width - bestObject.width) <= visionContinuityTolerance and
                abs(compObject.height - bestObject.height) <= visionContinuityTolerance):
            drivebase.drive(0, 0)
        elif abs(compObject.centerX - (cameraWidth / 2)) < fruitAlignmentTolerance:
            timeout = -1
            break
        else:
            drivebase.drive(0, (compObject.centerX - (cameraWidth / 2)) * -fruitAlignmentPID[0])

        timeout -= 1

    return timeout == 0            

# Harvest the fruit. This is somewhat implemented for teleop, but until the fruit detection fully worked 
# this step would not be possible
def Harvest():
    pass

# Back up from the tree without getting tangled. We do not trust our navigation function to pick 
# the right path from right under the tree, especially given the width of the robot
def Retreat():
    drivebase.moveLen(-6, 40)
    pass

binCoords = [0, 0, 0] #3rd is heading

# Move to the location of the bin (pre-staged in a certain spot), and then score the fruit
def Score():
    Navigate(binCoords[0], binCoords[1], binCoords[2])
    #Spin intake backwards
    pass

# Press the robot against the wall opposite from the ramp OR use the angle of april tags to 
# reset the heading from the gyro, as it will drift over the course of 15 minutes
def GyroCorrection():
    pass

def GroundDetect():
    pass

def GroundHarvest():
    pass

# Allow the code to reference the functions for each state just by passing the state number
stateFuncs = [Ramp, TreeDetection, FruitDetection, Harvest, Retreat, Score, GyroCorrection, GroundDetect, GroundHarvest]

currentState = 0

# Core state machine function. The states did not mature to the point of full implementation
def Run():
    stateFuncs[currentState]()

# Gets the largest object from the list of objects. By default is by the object height to make sure it does not bias towards
# the fat fruit
def LargestVisionObject(visionObjects, byArea = False):
    best = visionObjects[0]
    visionObjects = visionObjects[1:]
    if byArea:
        for visionObject in visionObjects:
            if visionObject.height * visionObject.width > best.height * best.width:
                best = visionObject
    else:
        for visionObject in visionObjects:
            if visionObject.height > best.height:
                best = visionObject
    return best

# Gets the index of the largest object
def LargestVisionObjectIndex(visionObjects, byArea = False):
    indexBest = 0
    i = 1

    if byArea:
        while i < visionObjects.count:
            if visionObjects[i].height * visionObjects[i].width > visionObjects[indexBest].height * visionObjects[indexBest].width:
                indexBest = i
            i += 1
    else:
        while i < visionObjects.count:
            if visionObjects[i].height > visionObjects[indexBest].height:
                indexBest = i
            i += 1

    return indexBest

# Our pathfinding function. To avoid the trees and ramps, we put 8 virtual navigation lines on the field.
# When the robot wants to navigate to any arbitrary point from another arbitrary point, it will enter on the closest
# of the 8 lines, and then travel on the lines to the linr closest to the end point, where it will then get off once
# it is as close as it can get to the final point while on the line. Functions are self explanatory
def Navigate(x, y, heading):
    # fieldX and fieldY are references from Localization.py
    # Our field positioning uses a combination of april tags, odometry, and the gyro
    currentLine = ClosestLine(fieldX, fieldY)
    targetLine = ClosestLine(x, y)
    targetLinePoint = ClosestPointOnLineSegment(targetLine, x, y)

    # Entrance to the next line
    entrance = ClosestPointOnLineSegment(currentLine, fieldX, fieldY)

    DriveToPoint(entrance[0], entrance[1])

    i = 0

    # LineNav is a 3d array, where the first axis is the line you start on, the second axis is the line you are going to,
    # and the 3rd axis is the list of lines that you travel on to reach the end line from the start line
    # LineConnections gives the point on the field where 2 lines connect to each other
    while i < len(LineNav[currentLine][targetLine]) - 1:
        entrance = LineConnections[LineNav[currentLine][targetLine][i]][LineNav[currentLine][targetLine][i + 1]]
        DriveToPoint(entrance[0], entrance[1])
        i += 1

    DriveToPoint(targetLinePoint[0], targetLinePoint[1])
    DriveToPoint(x, y)
    drivebase.turn(heading)

# Calculate the heading error while accounting for the jump from 359 - 0
def headingError(setpoint, current):
        err = setpoint - current
        while err < -180:
            err += 360
        while err > 180:
            err -= 360
        return err

def DriveToPoint(x, y):
    if not ((x - fieldX)**2 + (y - fieldY)**2 > fieldSetpointTolerance**2):
        return
    drivebase.turn(HeadingToPoint(fieldX, fieldY, x, y))
    while (x - fieldX)**2 + (y - fieldY)**2 > fieldSetpointTolerance**2:
        print(headingError(HeadingToPoint(fieldX, fieldY, x, y), 360 - inertial.orientation(YAW)))
        drivebase.drive(fieldDriveSpeed, headingError(HeadingToPoint(fieldX, fieldY, x, y), 360 - inertial.orientation(YAW)) * fieldDriveHeadingPID[0])
    drivebase.drive(0, 0)

def DistToLineSegment(line, xp, yp):
    # If the dot product of the vector from one point of the line to the other and the vector from the point to our robot
    # is negative, then we know the closest point is that end point. Otherwise it would return a point as if the line was infinitely long
    if (Lines[line][2] - Lines[line][0]) * (Lines[line][2] - xp) + (Lines[line][1] - Lines[line][3]) * (Lines[line][1] - yp) < 0:
        return math.sqrt((Lines[line][2] - xp) ** 2 + (Lines[line][1] - yp) ** 2)
    elif (Lines[line][0] - Lines[line][2]) * (Lines[line][0] - xp) + (Lines[line][3] - Lines[line][1]) * (Lines[line][3] - yp) < 0:
        return math.sqrt((Lines[line][0] - xp) ** 2 + (Lines[line][3] - yp) ** 2)
    else:
        return (abs((Lines[line][3] - Lines[line][1]) * xp - (Lines[line][2] - Lines[line][0]) * 
                yp + Lines[line][2] * Lines[line][1] - Lines[line][3] * Lines[line][0]) / 
                math.sqrt((Lines[line][3] - Lines[line][1]) ** 2 + (Lines[line][2] - Lines[line][0]) ** 2))

def ClosestPointOnLineSegment(line, xp, yp):
    # The implementation only works for lines parallel to the x or y axis, as all nav lines are perfectly aligned with an axis
    if (Lines[line][1] - Lines[line][3]) == 0:
        return [max(min(Lines[line][0], Lines[line][2]), min(max(Lines[line][0], Lines[line][2]), xp)), Lines[line][1]]
    elif (Lines[line][0] - Lines[line][2]) == 0:
        return [Lines[line][0], max(min(Lines[line][1], Lines[line][3]), min(max(Lines[line][1], Lines[line][3]), yp))]
    return [0, 0]

def ClosestLine(xp, yp):
    lowestIndex = 0
    lowestDist = DistToLineSegment(0, xp, yp)
    i = 1
    while i < 8:
        checkDist = DistToLineSegment(i, xp, yp)
        if checkDist < lowestDist:
            lowestIndex = i
            lowestDist = checkDist
        i += 1

    return lowestIndex

def HeadingToPoint(x1, y1, x2, y2):
    return math.atan2(y2 - y1, x2 - x1) * 360 / (2 * math.pi)

def DistanceBetweenPoints(x1, y1, x2, y2):
    return math.sqrt((y1 - y2) * (y1 - y2) + (x1 - x2) * (x1 - x2))

# Runs the teleop code in an async manner as to not block inputs
driveThread = Thread(drivebase.controllerDrive)