import math

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
cameraFocalLength = 2.5 #inches
cameraFOV = 60 #degrees

fruitHeight1 = 5 #inches
fruitHeight2 = 10 #inches

smallFruitWidth = 2.5 #inches
largeFruitWidth = 5.5 #inches

objectThreashold = 5

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

minArmLength = 0 #inches
maxArmLength = 20 #inches

minArmAngle = 0 #degrees
maxArmAngle = 180 #degrees

minWristAngle = 0 #degrees
maxWristAngle = 180 #degrees

armGearRatio = 5
pivotGearRatio = 5
wristGearRatio = 5

armPID = [0,0,0]
armTolerance = 1
pivotPID = [0,0,0]
pivotTolerance = 1
wristPID = [0,0,0]
wristTolerance = 1