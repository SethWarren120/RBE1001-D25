# Library imports
from vex import *

from constants import *
from Subsystems.Drivebase.drivebaseMotorCorrector import *
from Subsystems.subsystem import Subsystem

class TankDrivebase (Subsystem):
    #initalization
    def __init__(self, _motorLeft: Motor, _motorRight: Motor, gyro: Inertial, vision: AiVision,
                 motorCorrectionConfig = None, _rotationUnits = DEGREES, _speedUnits = RPM):
        self.motorLeft = _motorLeft
        self.motorRight = _motorRight
        self.gyro = gyro
        self.vision = vision

        self.rotationUnits = _rotationUnits
        self.speedUnits = _speedUnits

        self.x = 0.0
        self.y = 0.0
        self.heading = 0.0

        odometryThread = Thread(self.updateOdometry)

        if motorCorrectionConfig == None:
            motorCorrectionConfig = DrivebaseMotorCorrectionProfile.Disabled(_rotationUnits)
        else:
            motorCorrectionConfig.units = _rotationUnits

        self.motorCorrector = DrivebaseMotorCorrector([_motorLeft, _motorRight], motorCorrectionConfig)

    #lab 1 move code
    def moveLen(self, len, speed):
        deg = 360 * ((len / wheelCircumference) * gear_ratio)

        self.motorCorrector.setPassiveMode(False)
        self.motorCorrector.correctMotors([deg, deg])

        self.motorLeft.spin_for(FORWARD, deg, self.rotationUnits, speed, self.speedUnits, False)
        self.motorRight.spin_for(FORWARD, deg, self.rotationUnits, speed, self.speedUnits, True)

    #lab 1 turn code
    def turnDeg(self, rotation, speed, pivotOffset = 0.0):
        lenLeft = -(2 * math.pi * ((wheel_base / 2) + pivotOffset)) * (-rotation / 360)
        lenRight = (2 * math.pi * ((wheel_base / 2) - pivotOffset)) * (-rotation / 360)

        degLeft = 360 * ((lenLeft / wheelCircumference) * gear_ratio)
        degRight = 360 * ((lenRight / wheelCircumference) * gear_ratio)

        self.motorCorrector.setPassiveMode(False)
        self.motorCorrector.correctMotors([degLeft, degRight])

        if (degLeft > degRight):
            self.motorLeft.spin_for(FORWARD, degLeft, DEGREES, speed, self.speedUnits, False)
            self.motorRight.spin_for(FORWARD, degRight, DEGREES, speed * (degRight / degLeft), self.speedUnits, True)
        else:
            self.motorLeft.spin_for(FORWARD, degLeft, DEGREES, speed * (degLeft / degRight), self.speedUnits, False)
            self.motorRight.spin_for(FORWARD, degRight, DEGREES, speed, self.speedUnits, True)

    #experimental odometry code
    def updateOdometry(self):
        leftDistance = self.motorLeft.position(DEGREES) / 360.0 * wheelCircumference
        rightDistance = self.motorRight.position(DEGREES) / 360.0 * wheelCircumference

        avgX = (leftDistance - rightDistance) / 4.0
        avgY = (leftDistance + rightDistance) / 4.0
        
        heading = self.gyro.heading(DEGREES)
        headingRadians = math.radians(heading)
        
        deltaX = avgX * math.cos(headingRadians) - avgY * math.sin(headingRadians)
        deltaY = avgX * math.sin(headingRadians) + avgY * math.cos(headingRadians)
        
        self.x += deltaX
        self.y += deltaY
        self.heading = self.gyro.heading(DEGREES)

    #drive command using joysticks
    def driveCommand(self, leftAxis, rightAxis):
        self.motorLeft.spin(FORWARD, leftAxis + rightAxis, self.speedUnits)
        self.motorRight.spin(FORWARD, leftAxis - rightAxis, self.speedUnits)
    
    #uses the camera to center the robot on an object
    def centerToObject(self):
        objects = None
        #loops until it finds an object
        while True:
            objects = self.vision.take_snapshot(vision_orange)
            if len(objects) > 0:
                break
        
        #gets the center x and y of the first object
        x = objects[0].centerX
        y = objects[0].centerY

        #the camera outputs with 0,0 being the top right of the camera
        #this code moves 0,0 to the center of the camera
        correctedX = x + cameraXOffset
        correctedY = y + cameraYOffset

        #drives the wheels to get the object to the center of the camera
        self.motorLeft.spin(FORWARD, correctedX*0.8, self.speedUnits)
        self.motorRight.spin(FORWARD, -correctedX*0.8, self.speedUnits)

    #uses the camera to stay a certain distance from an object
    def stayDistanceFromObject(self):
        objects = None
        while True:
            #loops until it finds an object
            objects = self.vision.take_snapshot(vision_orange)
            if len(objects) > 0:
                break
        
        #gets the width of the first object
        viewedWidth = objects[0].width
        #68 is the desired width of the object at the desired distance
        #gets the change in width that the robot needs to go
        widthChange = viewedWidth - 68

        #drives the robot forward or backwards to get the object to the desired width
        self.motorLeft.spin(FORWARD, widthChange*0.8, self.speedUnits)
        self.motorRight.spin(FORWARD, widthChange*0.8, self.speedUnits)
    
    def lab(self):
        #loops forever
        while True:
            #runs the center then runs the stay distance function
            self.centerToObject()
            # self.stayDistanceFromObject()
            wait(20)