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

vision_Yellow = Colordesc(2, 140, 106, 60, 7, 0.37)
vision_Green = Colordesc(3, 14, 143, 61, 15, 0.41)
vision_Orange = Colordesc(1, 232, 152, 138, 10, 0.12)
vision_Pink = Colordesc(3, 232, 78, 146, 11, 0.13)

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
visionContinuityTolerance = 5

fruitHeight1 = 5 #inches
fruitHeight2 = 10 #inches

#0, 0, 0 is the center of the field facing the ramp
#tag constants
#x y angle (inches, degrees)
tag1 = [0,0,0]
tag2 = [0,0,0]
tag3 = [0,0,0]
tag4 = [0,0,0]
tag5 = [0,0,0]
tag6 = [0,0,0]
tag7 = [0,0,0]
tag8 = [0,0,0]
tag9 = [0,0,0]
tag10 = [0,0,0]
tag11 = [0,0,0]
tag12 = [0,0,0]
tag13 = [0,0,0]
tag14 = [0,0,0]
tag15 = [0,0,0]
tag16 = [0,0,0]
tagLocations = [tag1,
                tag2,
                tag3,
                tag4,
                tag5,
                tag6,
                tag7,
                tag8,
                tag9,
                tag10,
                tag11,
                tag12,
                tag13,
                tag14,
                tag15,
                tag16]

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
pivotPID = [1.8,0,0]
pivotFF = 0.09
pivotTolerance = 0.2
wristPID = [1,0,0]
wristTolerance = 0.5

post1Height = [0, 25, 20]
post2Height = [0, 45, 45]
post3Height = [270, 46, 42]
post4Height = [270, 50, 55]

fruitAlignmentPID = [0.1, 0, 0]
fruitAlignmentTolerance = 10


# Field Navigation
Lines = [[0, 0, 36, 0], [36, 0, 36, 60], [36, 60, 0, 60], [0, 60, 0, 0], 
         [36, 20, 24, 20], [36, 40, 24, 40], [0, 40, 12, 40], [0, 20, 12, 20]]
LineConnections = [[-1, [36, 0], -1, [0, 0], -1, -1, -1, -1],
                   [[36, 0], -1, [36, 60], -1, [36, 20], [36, 40], -1, -1],
                   [-1, [36, 60], -1, [0, 60], -1, -1, -1, -1],
                   [[0, 0], -1, [0, 60], -1, -1, -1, [0, 40], [0, 20]],
                   [-1, [36, 20], -1, -1, -1, -1, -1, -1],
                   [-1, [36, 40], -1, -1, -1, -1, -1, -1],
                   [-1, -1, -1, [0, 40], -1, -1, -1, -1],
                   [-1, -1, -1, [0, 20], -1, -1, -1, -1]]
LineNav = [[[0], [1], [1, 2], [3], [1, 4], [1, 5], [3, 6], [3, 7]],
           [[0], [1], [2], [0, 3], [4], [5], [2, 3, 6], [0, 3, 7]],
           [[3, 0], [1], [2], [3], [1, 4], [1, 5], [3, 6], [3, 7]],
           [[0], [0, 1], [2], [3], [0, 1, 4], [2, 1, 5], [6], [7]],
           [[1, 0], [1], [1, 2], [1, 0, 3], [4], [1, 5], [1, 0, 3, 6], [1, 0, 3, 7]],
           [[1, 0], [1], [1, 2], [1, 2, 3], [1, 4], [5], [1, 2, 3, 6], [1, 2, 3, 7]],
           [[3, 0], [3, 2, 1], [3, 2], [3], [3, 2, 1, 4], [3, 2, 1, 5], [6], [3, 7]],
           [[3, 0], [3, 0, 1], [3, 2], [3], [3, 0, 1, 4], [3, 0, 1, 5], [3, 6], [7]]]