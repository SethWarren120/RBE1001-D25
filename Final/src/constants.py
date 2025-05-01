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

drivePID = [0.1,0,0]
turnPID = [0.1,0,0]

#vision constants
#x,y,z,roll,pitch,yaw
#x is forward, y is left, z is up
cameraOffset = [0, 0, 0, 0, 0, 0] #inches

vision_orange = Colordesc(1,245,134,89,5,0.17)
vision_yellow = Colordesc(1,245,134,89,5,0.17)
vision_green = Colordesc(1,245,134,89,5,0.17)

cameraWidth = 320
cameraHeight = 240
cameraXOffset = cameraWidth/2
cameraYOffset = cameraHeight/2

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

minArmLength = 2
maxArmLength = 108 

minArmAngle = 1
maxArmAngle = 70

minWristAngle = 2
maxWristAngle = 180

armGearRatio = 5
pivotGearRatio = 72/12 * 20/12
wristGearRatio = 5

armPID = [1,0,0]
armTolerance = 1
pivotPID = [7.5,0,0]
pivotFF = 1
pivotTolerance = 0.5
wristPID = [1,0,0]
wristTolerance = 1