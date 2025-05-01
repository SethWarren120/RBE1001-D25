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
#x is forward, y is left, z is up
vision_orange = Colordesc(1, 245, 134, 89, 9, 0.37)
cameraOffset = [0, 0, 0] #inches
cameraFOV = 75 #degrees

cameraXOffset = -320/2
cameraYOffset = -240/2

fruitHeight1 = 5 #inches
fruitHeight2 = 10 #inches

smallFruitWidth = 2 + 3/8 #inches
largeFruitWidth = 5.5 #inches

objectThreashold = 5