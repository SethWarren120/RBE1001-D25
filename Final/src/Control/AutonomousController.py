from enum import Enum
from constants import *
from src.Control.Orchard import *
from main import *

class AutoState(Enum):
    RAMP = 0
    TREEDETECTION = 1
    FRUITDETECTION = 2
    HARVEST = 3
    RETREAT = 4
    SCORE = 5  
    # Contingency States
    GYROCORRECTION = 6
    GROUNDDETECT = 7
    GROUNDHARVEST = 8  



def Ramp():

    pass

treeNumber = -1
fruitNumber = -1

def TreeDetection():
    pass

fruitColor = -1

def FruitDetection():
    timeout = 75

    continuityCounter = 0

    bestObjectColor = -1

    while timeout > 0:
        objectsGreen = camera.take_snapshot(vision_Green)
        objectsOrange = camera.take_snapshot(vision_Orange)
        objectsYellow = camera.take_snapshot(vision_Yellow)
        
        if objectsGreen.count == 0 and objectsOrange.count == 0 and objectsYellow.count == 0:
            timeout -= 1
            continue

        if 'bestObject' in locals() and not bestObjectColor == -1:
            if bestObjectColor == 0:
                compObject = LargestVisionObject(objectsGreen)
            elif bestObjectColor == 1:
                compObject = LargestVisionObject(objectsOrange)
            else:
                compObject = LargestVisionObject(objectsYellow)

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
                bestObjectColor = -1
                timeout -= 1
        else:
            bestObjects = [LargestVisionObject(objectsGreen), LargestVisionObject(objectsOrange), LargestVisionObject(objectsYellow)]
            bestObjectColor = LargestVisionObjectIndex(bestObjects)
            bestObject = bestObjects[bestObjectColor]

    timeout = 2000

    while (timeout > 0):
        if bestObjectColor == 0:
            compObject = LargestVisionObject(objectsGreen)
        elif bestObjectColor == 1:
            compObject = LargestVisionObject(objectsOrange)
        else:
            compObject = LargestVisionObject(objectsYellow)

        if not (abs(compObject.width - bestObject.width) <= visionContinuityTolerance and
                abs(compObject.height - bestObject.height) <= visionContinuityTolerance):
            drivebase.drive(0, 0)
        elif abs(compObject.centerX - (cameraWidth / 2)) < fruitAlignmentTolerance:
            timeout = -1
            break
        else:
            drivebase.drive(0, (compObject.centerX - (cameraWidth / 2)) * fruitAlignmentPID[0])

        timeout -= 1

    fruitColor = bestObjectColor

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

stateFuncs = {
    AutoState.RAMP              : Ramp,
    AutoState.TREEDETECTION     : TreeDetection,
    AutoState.FRUITDETECTION    : FruitDetection,
    AutoState.HARVEST           : Harvest,
    AutoState.RETREAT           : Retreat,
    AutoState.SCORE             : Score,
    AutoState.GYROCORRECTION    : GyroCorrection,
    AutoState.GROUNDDETECT      : GroundDetect,
    AutoState.GROUNDHARVEST     : GroundHarvest
}

currentState = AutoState.RAMP

def Run():
    stateFuncs[currentState]()

def LargestVisionObject(visionObjects, byArea = False):
    indexBest = 0
    i = 1

    if byArea:
        while i < visionObjects.count:
            if visionObjects[i].height * visionObjects[i].width > visionObjects[indexBest].height * visionObjects[indexBest].width:
                indexBest = i
    else:
        while i < visionObjects.count:
            if visionObjects[i].height > visionObjects[indexBest].height:
                indexBest = i

    return visionObjects[indexBest]

def LargestVisionObjectIndex(visionObjects, byArea = False):
    indexBest = 0
    i = 1

    if byArea:
        while i < visionObjects.count:
            if visionObjects[i].height * visionObjects[i].width > visionObjects[indexBest].height * visionObjects[indexBest].width:
                indexBest = i
    else:
        while i < visionObjects.count:
            if visionObjects[i].height > visionObjects[indexBest].height:
                indexBest = i

    return indexBest

def Navigate(x, y, heading):
    #Implement April tags and odom
    cx = 0
    cy = 0
    ch = 0
    currentLine = ClosestLine(cx, cy)
    targetLine = ClosestLine(x, y)
    targetLinePoint = ClosestPointOnLineSegment(targetLine, x, y)

    drivebase.turn(HeadingToPoint(cx, cy, x, y))
    drivebase.moveLen(DistToLineSegment(currentLine, cx, cy), DistToLineSegment(currentLine, cx, cy) * 40)

    i = 1

    while i < len(LineNav[currentLine][targetLine]) - 1:
        entrance = LineConnections[LineNav[currentLine][targetLine][i]][LineNav[currentLine][targetLine][i + 1]]
        drivebase.turn(HeadingToPoint(cx, cy, entrance[0], entrance[1]))
        drivebase.moveLen(DistanceBetweenPoints(entrance[0], entrance[1], cx, cy), DistanceBetweenPoints(entrance[0], entrance[1], cx, cy) * 40)

    drivebase.turn(HeadingToPoint(cx, cy, targetLinePoint[0], targetLinePoint[1]))
    drivebase.moveLen(DistToLineSegment(currentLine, cx, cy), DistToLineSegment(currentLine, cx, cy) * 40)

    drivebase.turn(HeadingToPoint(cx, cy, x, y))
    drivebase.moveLen(DistanceBetweenPoints(cx, cy, x, y), DistanceBetweenPoints(cx, cy, x, y) * 40) 
    drivebase.turn(heading)


def DistToLineSegment(line, xp, yp):
    dl = (abs((Lines[line][3] - Lines[line][2]) * xp - (Lines[line][1] - Lines[line][0]) * 
              yp + Lines[line][1] * Lines[line][2] - Lines[line][3] * Lines[line][0]) / 
          math.sqrt((Lines[line][3] - Lines[line][2]) ^ 2 + (Lines[line][1] - Lines[line][0]) ^ 2))
    #d1 = math.sqrt((yp - y1) * (yp - y1) + (xp - x1) * (xp - x1))
    #d2 = math.sqrt((yp - y2) * (yp - y2) + (xp - x2) * (xp - x2))
    #de = math.sqrt((y1 - y2) * (y1 - y2) + (x1 - x2) * (x1 - x2))

    return dl

def ClosestPointOnLineSegment(line, xp, yp):
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

    return lowestIndex

def HeadingToPoint(x1, y1, x2, y2):
    return math.atan2(y2 - y1, x2 - x1) * 360 / (2 * math.pi)

def DistanceBetweenPoints(x1, y1, x2, y2):
    return math.sqrt((y1 - y2) * (y1 - y2) + (x1 - x2) * (x1 - x2))