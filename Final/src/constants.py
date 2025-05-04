import math
from vex import *

#drive constants
wheelDiameter = 4.0
wheel_travel = math.pi*wheelDiameter
track_width = 11
wheel_base = 11
gear_ratio = 5
wheelCircumference = 3.14 * wheelDiameter 
degreesPerInch = 360.0 / wheelCircumference

drivePID = [0.01,0,0]

#vision constants
#x,y,z,roll,pitch,yaw
#x is forward, y is left, z is up
tagCameraOffset = [0, 0, 0, 0, 0, 0] #inches
objCameraOffset = [0, 0, 0, 0, 0, 0] #inches

vision_orange = Colordesc(1,245,134,89,5,0.17)
vision_yellow = Colordesc(1,245,134,89,5,0.17)
vision_green = Colordesc(1,245,134,89,5,0.17)

tagCameraWidth = 320
tagCameraHeight = 240
tagCameraXOffset = tagCameraWidth/2
tagCameraYOffset = tagCameraHeight/2

objCameraWidth = 320
objCameraHeight = 240
objCameraXOffset = objCameraWidth/2
objCameraYOffset = objCameraHeight/2

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

#x y (inches)
post1Location = [50,50]
post2Location = [50,50]
post3Location = [50,50]
post4Location = [50,50]
post5Location = [50,50]
post6Location = [50,50]
post7Location = [50,50]
post8Location = [50,50]
post9Location = [50,50]
posts = [post1Location, post2Location, post3Location,
         post4Location, post5Location, post6Location,
         post7Location, post8Location, post9Location]

#roller constants
rollerDiameter = 2.5 #inches
rollerGearRatio = 5
forksGearRatio = 5

#arm constants
lowFruitHeight = 5 #inches
lowFruitAngle = 45 #degrees
highFruitHeight = 10 #inches
highFruitAngle = 90 #degrees
lowFruitWristAngle = 5 #degrees
highFruitWristAngle = 10 #degrees

minArmLength = 0
maxArmLength = 270

minArmAngle = 1
maxArmAngle = 90

minWristAngle = 2
maxWristAngle = 180

armGearRatio = 12/6
pivotGearRatio = 72/12
wristGearRatio = 10/6

armPID = [1.5,0,0]
armFF = 0.5
armTolerance = 0.5
pivotPID = [1.8,0,0]
pivotFF = 0.5
pivotTolerance = 0.1
wristPID = [1,0,0]
wristTolerance = 1


post1Height = [0, 25, 20]
post2Height = [0, 45, 45]
post3Height = [270, 46, 42]
post4Height = [270, 55, 55]