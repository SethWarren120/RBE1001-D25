# Library imports
from vex import *
from Subsystems.Drivebase.drivebaseMotorCorrector import *

class TankDrivebase ():
    def __init__(self, _motorLeft, _motorRight, robotConfig,
                 motorCorrectionConfig = None, _rotationUnits = DEGREES, _speedUnits = RPM, kP = 0.5):
        self.motorLeft = _motorLeft
        self.motorRight = _motorRight

        self.diameter = robotConfig.getWheelDiameter()
        self.gearing = robotConfig.getGearRatio()
        self.wheelBase = robotConfig.getDrivebaseWidth()
        self.circumference = robotConfig.getCircumference()
        self.dpi = robotConfig.getDPI()

        self.rotationUnits = _rotationUnits
        self.speedUnits = _speedUnits
        self.kP = kP

        self.rangeFinderRightSide = robotConfig.getRightRangeFinder()
        self.rangeFinderFront = robotConfig.getFrontRangeFinder()
        self.inertial = robotConfig.getInertialSensor()
        self.leftLineSensor = robotConfig.getLeftLineSensor()
        self.rightLineSensor = robotConfig.getRightLineSensor()
        self.bumpSwitch = robotConfig.getBumpSwitch()
        self.armMotor = robotConfig.getArmMotor()

        self.desiredDistance = 8.0

        if motorCorrectionConfig == None:
            motorCorrectionConfig = DrivebaseMotorCorrectionProfile.Disabled(_rotationUnits)
        else:
            motorCorrectionConfig.units = _rotationUnits

        self.motorCorrector = DrivebaseMotorCorrector([_motorLeft, _motorRight], motorCorrectionConfig)

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

    
    def drive(self, speed, direction):
        left_speed = speed - direction
        right_speed = speed + direction

        self.motorLeft.spin(FORWARD,left_speed)
        self.motorRight.spin(FORWARD,right_speed)

    def wallFollowInches(self, setDistanceFromWall):
        rightError = self.rangeFinderRightSide.distance(INCHES) - setDistanceFromWall
        self.drive(200, -self.kP*rightError)

    def hitWall(self):
        return self.rangeFinderFront.distance(INCHES) < self.desiredDistance
    
    def onLine(self, sensor):
        whiteLineValue = 94
        return sensor.reflectivity() < whiteLineValue
    

    def driveLab21(self):
        # drive until the first wall
        self.desiredDistance = 8.0
        while not self.hitWall():
            self.wallFollowInches(5.0)
            wait(20)

        self.motorLeft.stop()
        self.motorRight.stop()
        
        self.motorRight.reset_position()
        self.motorLeft.reset_position()
        #turn left 90 degrees
        while abs(self.motorLeft.position(self.rotationUnits))/5 < (self.wheelBase+0.8)/4 * 90 + 5:
            self.drive(0,200)
        
        self.motorLeft.stop()
        self.motorRight.stop()
        
        wait(20)

        self.desiredDistance = 10
        while not self.hitWall():
            self.wallFollowInches(7.0)
            wait(20)
        
        self.motorLeft.stop()
        self.motorRight.stop()

        self.motorRight.reset_position()
        self.motorLeft.reset_position()

        wait(20)
        
        #drive backwards
        while abs(self.motorLeft.position(self.rotationUnits)/5) < 880:
            self.drive(-200,0)
        
        self.motorLeft.stop()
        self.motorRight.stop()

        self.motorRight.reset_position()
        self.motorLeft.reset_position()
        #turn left 90 degrees
        while abs(self.motorLeft.position(self.rotationUnits))/5 < (self.wheelBase+0.8)/4 * 90 + 5:
            self.drive(0,200)

        self.motorLeft.stop()
        self.motorRight.stop()

        wait(20)

        self.motorLeft.reset_position()
        #drive forward
        while self.motorLeft.position(self.rotationUnits)/5 < 550:
            self.drive(200,0)

        self.motorLeft.stop()
        self.motorRight.stop()

    def driveLab22(self):
        self.desiredDistance = 8
        #drive until first wall
        while not self.hitWall():
            self.wallFollowInches(5.0)
            wait(20)
        
        self.motorLeft.stop()
        self.motorRight.stop()

        self.motorRight.reset_position()
        self.motorLeft.reset_position()

        wait(20)

        self.inertial.set_heading(180)

        #turn left 90 degrees
        while abs(self.inertial.heading()) > 110:
            self.drive(0,200)
            wait(20)

        self.motorLeft.stop()
        self.motorRight.stop()

        self.desiredDistance = 10
        #drive until second wall
        while not self.hitWall():
            self.wallFollowInches(11.0)
            wait(20)
        
        self.inertial.set_heading(180)

        wait(20)

        self.motorRight.reset_position()
        self.motorLeft.reset_position()
        
        #drive backwards
        while abs(self.motorLeft.position(self.rotationUnits)/5) < 880:
            self.drive(-200,0)
        
        self.motorLeft.stop()
        self.motorRight.stop()

        #turn left 90 degrees
        while abs(self.inertial.heading()) > 110:
            self.drive(0,200)
            wait(20)

        self.inertial.set_heading(180)

        self.motorLeft.stop()
        self.motorRight.stop()

        self.motorRight.reset_position()
        self.motorLeft.reset_position()

        wait(20)

        #drive forward
        while abs(self.motorLeft.position(self.rotationUnits)/5) < 550:
            self.drive(200,0)

        self.motorLeft.stop()
        self.motorRight.stop()

    def driveLab23(self):
        self.desiredDistance = 3
        #use the line sensors to follow the line until the first wall
        #should turn more the longer it is off the line
        while not self.hitWall():
            rightError = self.rangeFinderRightSide.distance(INCHES) - 6
            if (not self.onLine(self.leftLineSensor)):
                self.drive(200, 20)
            elif (not self.onLine(self.rightLineSensor)):
                self.drive(200, -20)
            else:
                self.drive(200, -self.kP*rightError)
            wait(20)

        self.motorLeft.stop()
        self.motorRight.stop()

        self.motorRight.reset_position()
        self.motorLeft.reset_position()

        wait(20)

        self.inertial.set_heading(180)

        #turn left 90 degrees
        while abs(self.inertial.heading()) > 110:
            self.drive(0,200)
            wait(20)

        
        self.drive(0,200)

        self.motorLeft.stop()
        self.motorRight.stop()

        self.desiredDistance = 8
        #drive until the second wall
        while not self.hitWall():
            rightError = self.rangeFinderRightSide.distance(INCHES) - 6
            if (not self.onLine(self.leftLineSensor)):
                self.drive(200, 20)
            elif (not self.onLine(self.rightLineSensor)):
                self.drive(200, -20)
            else:
                self.drive(200, -self.kP*rightError)
            wait(20)

        self.motorLeft.stop()
        self.motorRight.stop()

        self.motorRight.reset_position()
        self.motorLeft.reset_position()

        wait(20)

        #back up
        while abs(self.motorLeft.position(self.rotationUnits)/5) < 880:
            rightError = self.rangeFinderRightSide.distance(INCHES) - 6
            if (not self.onLine(self.leftLineSensor)):
                self.drive(-200, -20)
            elif (not self.onLine(self.rightLineSensor)):
                self.drive(-200, 20)
            else:
                self.drive(-200, -self.kP*rightError)
            wait(20)

        self.motorLeft.stop()
        self.motorRight.stop()

        self.motorRight.reset_position()
        self.motorLeft.reset_position()

        wait(20)
            
        self.inertial.set_heading(180)

        #turn left 90 degrees
        while abs(self.inertial.heading()) > 110:
            self.drive(0,200)
            wait(20)

        self.inertial.set_heading(180)

        self.motorLeft.stop()
        self.motorRight.stop()

        self.motorRight.reset_position()
        self.motorLeft.reset_position()

        wait(20)

        #drive forward
        while abs(self.motorLeft.position(self.rotationUnits)/5) < 550:
            self.drive(200,0)

        self.motorLeft.stop()
        self.motorRight.stop()