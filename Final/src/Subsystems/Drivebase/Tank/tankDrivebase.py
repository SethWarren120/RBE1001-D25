# Library imports
from vex import *
from Subsystems.Drivebase.drivebaseMotorCorrector import *
from constants import *

class TankDrivebase ():

    diameter = 4
    gearing = 5 / 1
    wheelBase = 11
    circumference = math.pi * diameter
    
    def __init__(self, _motorLeft: Motor, _motorRight: Motor, gyro: Inertial, camera: AiVision, ultraSonic: Sonar, controller: Controller,
                 motorCorrectionConfig = None, _rotationUnits = DEGREES, _speedUnits = RPM):
        self.motorLeft = _motorLeft
        self.motorRight = _motorRight

        self.ultraSonic = ultraSonic
        self.camera = camera
        self.controller = controller

        self.diameter = wheelDiameter
        self.gearing = gear_ratio
        self.wheelBase = track_width
        self.circumference = math.pi * wheelDiameter
        self.dpi = 360.0 / self.circumference

        self.x = 0.0
        self.y = 0.0
        self.heading = 0.0

        self.gyro = gyro

        self.rotationUnits = _rotationUnits
        self.speedUnits = _speedUnits

        if motorCorrectionConfig == None:
            motorCorrectionConfig = DrivebaseMotorCorrectionProfile.Disabled(_rotationUnits)
        else:
            motorCorrectionConfig.units = _rotationUnits

        self.motorCorrector = DrivebaseMotorCorrector([_motorLeft, _motorRight], motorCorrectionConfig)
        
        driveThread = Thread(self.controllerDrive)
    
    def drive(self, speed, direction):
        left_speed = speed - direction
        right_speed = speed + direction

        self.motorLeft.spin(FORWARD,left_speed)
        self.motorRight.spin(FORWARD,right_speed)

    def controllerDrive(self):
        while True:
            speed = math.copysign(math.pow(self.controller.axis3.value()/3,2), self.controller.axis3.value())/9
            rot = -math.copysign(math.pow(self.controller.axis1.value()/3,2),self.controller.axis1.value())/9

            if (abs(speed) < 5):
                speed = 0
            if (abs(rot) < 5):
                rot = 0

            self.drive(speed, rot)
            wait(20)

    def moveLen(self, len, speed):
        deg = 360 * ((len / self.circumference) * self.gearing)

        self.motorCorrector.setPassiveMode(False)
        self.motorCorrector.correctMotors([deg, deg])

        self.motorLeft.spin_for(FORWARD, deg, self.rotationUnits, speed, self.speedUnits, False)
        self.motorRight.spin_for(FORWARD, deg, self.rotationUnits, speed, self.speedUnits, True)

    def turnDeg(self, rotation, speed, pivotOffset = 0.0):
        lenLeft = -(2 * math.pi * ((self.wheelBase / 2) + pivotOffset)) * (-rotation / 360)
        lenRight = (2 * math.pi * ((self.wheelBase / 2) - pivotOffset)) * (-rotation / 360)

        degLeft = 360 * ((lenLeft / self.circumference) * self.gearing)
        degRight = 360 * ((lenRight / self.circumference) * self.gearing)

        self.motorCorrector.setPassiveMode(False)
        self.motorCorrector.correctMotors([degLeft, degRight])

        if (degLeft > degRight):
            self.motorLeft.spin_for(FORWARD, degLeft, DEGREES, speed, self.speedUnits, False)
            self.motorRight.spin_for(FORWARD, degRight, DEGREES, speed * (degRight / degLeft), self.speedUnits, True)
        else:
            self.motorLeft.spin_for(FORWARD, degLeft, DEGREES, speed * (degLeft / degRight), self.speedUnits, False)
            self.motorRight.spin_for(FORWARD, degRight, DEGREES, speed, self.speedUnits, True)
    
    def updateOdometry(self):
        while True:
            leftDistance = self.motorLeft.position(DEGREES) / 360.0 * self.circumference
            rightDistance = self.motorRight.position(DEGREES) / 360.0 * self.circumference

            avgDistance = (leftDistance + rightDistance) / 2.0

            headingRadians = math.radians(self.gyro.heading(DEGREES))

            deltaX = avgDistance * math.cos(headingRadians)
            deltaY = avgDistance * math.sin(headingRadians)

            self.x += deltaX
            self.y += deltaY
            self.heading = self.gyro.heading(DEGREES)

            aprilTags = self.camera.take_snapshot(AiVision.ALL_TAGS)
            for tag in aprilTags:
                tagX, tagY, tagAngle = tagLocations[tag.id-1]
                observedX = tag.centerX
                observedY = tag.centerY
                observedAngle = tag.angle
                
                adjustedX = tagX - (tagCameraOffset[0] * math.cos(math.radians(self.heading)) - tagCameraOffset[1] * math.sin(math.radians(self.heading)))
                adjustedY = tagY - (tagCameraOffset[0] * math.sin(math.radians(self.heading)) + tagCameraOffset[1] * math.cos(math.radians(self.heading)))
                
                self.x += (self.x - (adjustedX - observedX))/2
                self.y += (self.y - (adjustedY - observedY))/2
                self.heading = (tagAngle - observedAngle) % 360