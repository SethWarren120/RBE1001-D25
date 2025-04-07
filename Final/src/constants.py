import math

wheelDiameter = 4.0
wheel_travel = math.pi*wheelDiameter
track_width = 11
wheel_base = 11
gear_ratio = 5
wheelCircumference = 3.14 * wheelDiameter 
degreesPerInch = 360.0 / wheelCircumference

drivePID = [1,0,0]
turnPID = [1,0,0]

fruitHeight1 = 5 #inches
fruitHeight2 = 10 #inches

smallFruitWidth = 2.5 #inches
largeFruitWidth = 5.5 #inches

objectThreashold = 5