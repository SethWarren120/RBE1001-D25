import math
from vex import *

#drive constants
wheelDiameter = 4.0
wheel_travel = math.pi*wheelDiameter
track_width = 15
wheel_base = 9.5
gear_ratio = 1
wheelCircumference = 3.14 * wheelDiameter 
degreesPerInch = 360.0 / wheelCircumference

drivePID = [0.01,0,0]

#vision constants
#x,y,z,roll,pitch,yaw
#x is forward, y is left, z is up
tagCameraOffset = [0, 0, 0, 0, 0, 0] #inches
objCameraOffset = [0, 0, 0, 0, 0, 0] #inches

vision_Green = Colordesc(1, 14, 143, 61, 15, 0.41)
vision_Yellow = Colordesc(2, 140, 106, 60, 7, 0.37)
vision_Orange = Colordesc(3, 232, 152, 138, 10, 0.12)
vision_Pink = Colordesc(4, 232, 78, 146, 11, 0.13)

vision_GreenBox = Codedesc(1, vision_Pink, vision_Green)
vision_YellowBox = Codedesc(2, vision_Pink, vision_Yellow)
vision_OrangeBox = Codedesc(3, vision_Pink, vision_Orange)

cameraWidth = 320
cameraHeight = 240
cameraXOffset = cameraWidth/2
cameraYOffset = cameraHeight/2
cameraFOV = 61 #degrees
cameraVerticalFOV = 41 #degrees

visionFruitSizeThreshold = 30
visionContinuityTolerance = 30

fruitHeight1 = 5 #inches
fruitHeight2 = 10 #inches

#0, 0, 0 is the center of the field facing the ramp
#tag constants
#x y angle (inches, degrees)
tagLocations = [[13, -0.75, 90],
                [86, -0.75, 90],
                [98.75, 13, 180],
                [98.75, 50, 180],
                [98.75, 90, 180],
                [98.75, 133, 180],
                [86, 146.75, 270],
                [13, 146.75, 270],
                [-0.75, 133, 0],
                [-0.75, 90, 0],
                [-0.75, 50, 0],
                [-0.75, 13, 0]]

minArmLength = 0
maxArmLength = 270

minArmAngle = 1
maxArmAngle = 90

armGearRatio = 12/6
pivotGearRatio = 72/12
wristGearRatio = 10/6

armPID = [1.5,0,0]
armFF = 0.1
armTolerance = 0.5
pivotPID = [1,0,0]
pivotFF = 0.09
pivotTolerance = 0.2
pivotMaxSpeed = 15
wristPID = [1,0,0]
wristTolerance = 0.5

post1Height = [180, 25, 20]
post2Height = [180, 50, 50]
post3Height = [140, 42, 42]
post4Height = [270, 52, 57]

fruitAlignmentPID = [0.1, 0, 0]
fruitAlignmentTolerance = 10

# Field Navigation
Lines = [[13, 13, 86, 13], [86, 13, 86, 133], [86, 133, 13, 133], [13, 133, 13, 13], 
         [86, 50, 24, 50], [86, 90, 24, 90], [13, 90, 12, 90], [13, 50, 12, 50]]
LineConnections = [[-1, [86, 13], -1, [13, 13], -1, -1, -1, -1],
                   [[86, 13], -1, [86, 133], -1, [86, 50], [86, 90], -1, -1],
                   [-1, [86, 133], -1, [13, 133], -1, -1, -1, -1],
                   [[13, 13], -1, [13, 133], -1, -1, -1, [13, 90], [13, 50]],
                   [-1, [86, 50], -1, -1, -1, -1, -1, -1],
                   [-1, [86, 90], -1, -1, -1, -1, -1, -1],
                   [-1, -1, -1, [13, 90], -1, -1, -1, -1],
                   [-1, -1, -1, [13, 50], -1, -1, -1, -1]]
LineNav = [[[0], [0,1], [0,1, 2], [0,3], [0,1, 4], [0,1, 5], [0,3, 6], [0,3, 7]],
           [[1,0], [1], [1,2], [1,0, 3], [1,4], [1,5], [1,2, 3, 6], [1,0, 3, 7]],
           [[2,3, 0], [2,1], [2], [2,3], [2,1, 4], [2,1, 5], [2,3, 6], [2,3, 7]],
           [[3,0], [3,0, 1], [3,2], [3], [3,0, 1, 4], [3,2, 1, 5], [3,6], [3,7]],
           [[4,1, 0], [4,1], [4,1, 2], [4,1, 0, 3], [4], [4,1, 5], [4,1, 0, 3, 6], [4,1, 0, 3, 7]],
           [[5,1, 0], [5,1], [5,1, 2], [5,1, 2, 3], [5,1, 4], [5], [5,1, 2, 3, 6], [5,1, 2, 3, 7]],
           [[6,3, 0], [6,3, 2, 1], [6,3, 2], [6,3], [6,3, 2, 1, 4], [6,3, 2, 1, 5], [6], [6,3, 7]],
           [[7,3, 0], [7,3, 0, 1], [7,3, 2], [7,3], [7,3, 0, 1, 4], [7,3, 0, 1, 5], [7,3, 6], [7]]]

ultrasonicWallClearance = 3.5
ultrasonicWallFollowPID = [5, 0, 0]

headingSetpointTolerance = 3
headingSetpointPID = [0.6, 0, 0]

fieldSetpointTolerance = 2
fieldDriveHeadingPID = [1, 0, 0]
fieldDriveSpeed = 30