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

# inertial.set_heading(180, DEGREES)
# inertial.calibrate()

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

# class AutoState(Enum):
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


controller.buttonR2.pressed(grabFruit)
controller.buttonR2.released(intake.stopIntake)

controller.buttonL2.pressed(arm.flipWrist)

controller.buttonB.pressed(dumpObject)

controller.buttonUp.pressed(lambda: arm.setSetpoint(post4Height[0], post4Height[1], post4Height[2]))
controller.buttonDown.pressed(lambda: arm.setSetpoint(post2Height[0], post2Height[1], post2Height[2]))
controller.buttonLeft.pressed(lambda: arm.setSetpoint(post3Height[0], post3Height[1], post3Height[2]))
controller.buttonRight.pressed(lambda: arm.setSetpoint(post1Height[0], post1Height[1], post1Height[2]))

controller.buttonX.pressed(arm.stowArm)

# controller.buttonY.pressed(lambda: arm.setSetpoint(180, 0, 90))

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

treeNumber = 2
fruitNumber = -1

def TreeDetection():
    pass

fruitColor = 0
tracking = False

def FruitDetection():

    arm.setSetpoint(0, 30, 0)
    drivebase.drive(0, 0)

    # if treeNumber >= 0 and treeNumber <= 2:
    #     fruitColor = 0
    # elif treeNumber >= 3 and treeNumber <= 5:
    #     fruitColor = 2
    # else:
    #     fruitColor = 1

    print("Fc")
    print(fruitColor)

    timeout = 75

    continuityCounter = 0

    while timeout > 0:
        if fruitColor == 0:
            objects = camera.take_snapshot(vision_Green)
        elif fruitColor == 1:
            objects = camera.take_snapshot(vision_Orange)
        else:
            objects = camera.take_snapshot(vision_Yellow)

        print(len(objects))
        print("fc")
        print(fruitColor)
        
        if len(objects) == 0:
            # timeout -= 1
            continue

        if 'bestObject' in locals() and tracking:
            compObject = LargestVisionObject(objects)
            print("Yes")
            
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
                # timeout -= 1
        else:
            bestObject = LargestVisionObject(objects)
            tracking = True

    print("Stable Object")

    timeout = 2000

    while (timeout > 0):
        if fruitColor == 0:
            compObject = LargestVisionObject(camera.take_snapshot(vision_Green))
        elif fruitColor == 1:
            compObject = LargestVisionObject(camera.take_snapshot(vision_Orange))
        else:
            compObject = LargestVisionObject(camera.take_snapshot(vision_Yellow))

        # if not (abs(compObject.width - bestObject.width) <= visionContinuityTolerance and
        #         abs(compObject.height - bestObject.height) <= visionContinuityTolerance):
        #     drivebase.drive(0, 0)
        # elif abs(compObject.centerX - (cameraWidth / 2)) < fruitAlignmentTolerance:
        #     timeout = -1
        #     break
        # else:
        drivebase.drive(0, (compObject.centerX - (cameraWidth / 2)) * -fruitAlignmentPID[0])

        timeout -= 1

    return timeout == 0            

def Harvest():
    pass

def Retreat():
    drivebase.moveLen(-6, 40)
    pass

binCoords = [0, 0, 0] #3rd is heading

def Score():
    Navigate(binCoords[0], binCoords[1], binCoords[2])
    #Spin intake backwards
    pass

def GyroCorrection():
    pass

def GroundDetect():
    pass

def GroundHarvest():
    pass

stateFuncs = [Ramp, TreeDetection, FruitDetection, Harvest, Retreat, Score, GyroCorrection, GroundDetect, GroundHarvest]

currentState = 0

def Run():
    stateFuncs[currentState]()

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

def Navigate(x, y, heading):
    #Implement April tags and odom
    print("Navigating")
    currentLine = ClosestLine(fieldX, fieldY)
    targetLine = ClosestLine(x, y)
    targetLinePoint = ClosestPointOnLineSegment(targetLine, x, y)

    entrance = ClosestPointOnLineSegment(currentLine, fieldX, fieldY)
    print(currentLine)
    print(targetLine)
    print(LineNav[currentLine][targetLine])
    print(entrance)

    DriveToPoint(entrance[0], entrance[1])

    i = 0

    while i < len(LineNav[currentLine][targetLine]) - 1:
        entrance = LineConnections[LineNav[currentLine][targetLine][i]][LineNav[currentLine][targetLine][i + 1]]
        print(entrance)
        DriveToPoint(entrance[0], entrance[1])
        i += 1

    print(targetLinePoint)

    DriveToPoint(targetLinePoint[0], targetLinePoint[1])
    DriveToPoint(x, y)
    drivebase.turn(heading)

def headingError(setpoint, current):
        err = setpoint - current
        while err < -180:
            err += 360
        while err > 180:
            err -= 360
        return err

def DriveToPoint(x, y):
    print("Driving")
    if not ((x - fieldX)**2 + (y - fieldY)**2 > fieldSetpointTolerance**2):
        return
    drivebase.turn(HeadingToPoint(fieldX, fieldY, x, y))
    while (x - fieldX)**2 + (y - fieldY)**2 > fieldSetpointTolerance**2:
        print(headingError(HeadingToPoint(fieldX, fieldY, x, y), 360 - inertial.orientation(YAW)))
        drivebase.drive(fieldDriveSpeed, headingError(HeadingToPoint(fieldX, fieldY, x, y), 360 - inertial.orientation(YAW)) * fieldDriveHeadingPID[0])
    print("Done Driving")
    drivebase.drive(0, 0)

def DistToLineSegment(line, xp, yp):
    if (Lines[line][2] - Lines[line][0]) * (Lines[line][2] - xp) + (Lines[line][1] - Lines[line][3]) * (Lines[line][1] - yp) < 0:
        return math.sqrt((Lines[line][2] - xp) ** 2 + (Lines[line][1] - yp) ** 2)
    elif (Lines[line][0] - Lines[line][2]) * (Lines[line][0] - xp) + (Lines[line][3] - Lines[line][1]) * (Lines[line][3] - yp) < 0:
        return math.sqrt((Lines[line][0] - xp) ** 2 + (Lines[line][3] - yp) ** 2)
    else:
        return (abs((Lines[line][3] - Lines[line][1]) * xp - (Lines[line][2] - Lines[line][0]) * 
                yp + Lines[line][2] * Lines[line][1] - Lines[line][3] * Lines[line][0]) / 
                math.sqrt((Lines[line][3] - Lines[line][1]) ** 2 + (Lines[line][2] - Lines[line][0]) ** 2))

def ClosestPointOnLineSegment(line, xp, yp):
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

Ramp()

fieldX = 5
fieldY = 13

driveThread = Thread(drivebase.controllerDrive)