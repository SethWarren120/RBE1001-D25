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

drivePID = [1,0,0]
turnPID = [1,0,0]

#vision constants
#x,y,z,roll,pitch,yaw
#x is forward, y is left, z is up
vision_orange = Colordesc(1,245,134,89,5,0.17)
cameraOffset = [0, 0, 0, 0, 0, 0] #inches
cameraFOV = 75 #degrees

cameraWidth = 320
cameraHeight = 240
cameraXOffset = -cameraWidth/2
cameraYOffset = -cameraHeight/2

fruitHeight1 = 5 #inches
fruitHeight2 = 10 #inches

smallFruitWidth = 2.5 #inches
largeFruitWidth = 5.5 #inches

objectThreashold = 5

#0, 0, 0 is the center of the field facing the ramp
#tag constants
#x y angle (inches, degrees)
tag1 = [0,0,0]
tag2 = [0,0,0]
tag3 = [0,0,0]
tag4 = [0,0,0]
tagLocations = [tag1,
                tag2,
                tag3,
                tag4]

#x y (inches)
post1Location = [0,0]
post2Location = [0,0]
post3Location = [0,0]
posts = [post1Location,
          post2Location,
          post3Location]

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
maxArmLength = 108 

minArmAngle = 0
maxArmAngle = 117

minWristAngle = 0
maxWristAngle = 180

armGearRatio = 5
pivotGearRatio = 72/12
wristGearRatio = 5

armPID = [1,0,0]
armTolerance = 1
pivotPID = [1.4,0,0]
pivotFF = 0.5
pivotTolerance = 0.5
wristPID = [1,0,0]
wristTolerance = 1