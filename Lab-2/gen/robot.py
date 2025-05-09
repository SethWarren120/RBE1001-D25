# Automatically generated deployment script
# Edits to this file will be overwritten
from vex import *
class DrivebaseMotorCorrector:
    print("hello :3")
    motors = []
    offset = 1
    offsetList = []
    tolerance = 5
    units = DEGREES
    passive = False
    passiveActive = False
    motorSnapshot = []
    loopTime = 20
    enabled = True
    configured = True
    def __init__(self, motorList, config):
        if len(motorList) != len(config.startingOffsets):
            raise Exception("Starting offset list does not contain the same number of entries as motors")
        self.motors = motorList
        self.offset = config.offset
        self.offsetList = config.startingOffsets
        self.tolerance = config.tolerance
        self.units = config.units
        self.loopTime = config.loopTime
        self.enabled = config.enabled
        self.configured = config.configured
    def setEnabled(self, enabledState, motorOffsetList = None):
        if not self.configured:
            raise Exception("Cannot enable Motor Correction unless it is configured")
        self.enabled = enabledState
        if motorOffsetList == None:
            self.offsetList = [0] * len(self.motors)
            self.correctMotors([1] * len(self.motors))
        else:
            self.offsetList = motorOffsetList
    def setPassiveMode(self, usePassiveMode):
        if not self.enabled:
            return
        self.passive = usePassiveMode
        if (usePassiveMode and ((not hasattr(self, 'passiveCorrectionTask')) or
                                     (hasattr(self, 'passiveCorrectionTask') and not self.passiveActive))):
            self.motorSnapshot = self.__getMotorSnapshot()
            self.passiveCorrectionTask = Thread(self.__passiveCorrectionLoop)
        elif not usePassiveMode:
            while hasattr(self, 'passiveCorrectionTask') and self.passiveActive:
                sleep(self.loopTime / 4)
    def correctMotors(self, *travelDirections):
        if not self.enabled:
            return
        if len(travelDirections) != len(self.motors):
            raise Exception("Travel direction list does not contain the same number of entries as motors")
        if self.passive:
            raise Exception("Cannot use Manual Correction Mode while Self Correction Mode is active")
        for i in range(len(travelDirections)):
            if abs(self.offset - self.offsetList[i] if travelDirections[i] >= 0 else -self.offsetList[i]) < self.tolerance:
                continue
            self.motors[i].spin_for(FORWARD, self.offset - self.offsetList[i] if travelDirections[i] >= 0 else -self.offsetList[i], 
                                    self.units, 200, RPM, False)
        block = True
        while block:
            spinning = False
            for motor in self.motors:
                if motor.is_spinning():
                    spinning = True
            if spinning:
                sleep(self.loopTime)
            else:
                block = False
    def __passiveCorrectionLoop(self):
        self.passiveActive = True
        while self.passive:
            sleep(self.loopTime)
            snapshot = self.__getMotorSnapshot()
            for i in range(len(snapshot)):
                self.offsetList[i] = max(0, min(self.offset, self.offsetList[i] + (snapshot[i] - self.motorSnapshot[i])))
            self.motorSnapshot = snapshot
        self.passiveActive = False
    def __getMotorSnapshot(self):
        snapshot = []
        for motor in self.motors:
            snapshot.append(motor.get_position())
        return snapshot
    def getPosition(self, motorIndex, rotationUnits = None):        
        if rotationUnits == None:
            rotationUnits = self.units
        return self.motors[motorIndex].get_position(rotationUnits) - self.offsetList[motorIndex] if self.enabled else 0
class DrivebaseMotorCorrectionProfile:
    offset = 1
    startingOffsets = []
    tolerance = 5
    passive = False
    units = DEGREES
    loopTime = 20
    enabled = True
    configured = True
    def __init__(self, offsetAmount, startingOffsetList, correctionTolerance, passiveMode = False,
                 correctionEnabled = True, rotationUnits = DEGREES, correctionLoopTime = 20):
        self.offset = offsetAmount
        self.startingOffsets = startingOffsetList
        self.tolerance = correctionTolerance
        self.passive = passiveMode
        self.units = rotationUnits
        self.loopTime = correctionLoopTime
        self.enabled = correctionEnabled
        self.configured = True
    @staticmethod
    def Disabled(rotationUnits = DEGREES):
        config = DrivebaseMotorCorrectionProfile(0, [0, 0], 0, False, False, rotationUnits)
        print(config.startingOffsets)
        config.configured = False
        return config
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
        self.desiredDistance = 8.0
        while not self.hitWall():
            self.wallFollowInches(5.0)
            wait(20)
        self.motorLeft.stop()
        self.motorRight.stop()
        self.motorRight.reset_position()
        self.motorLeft.reset_position()
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
        while abs(self.motorLeft.position(self.rotationUnits)/5) < 880:
            self.drive(-200,0)
        self.motorLeft.stop()
        self.motorRight.stop()
        self.motorRight.reset_position()
        self.motorLeft.reset_position()
        while abs(self.motorLeft.position(self.rotationUnits))/5 < (self.wheelBase+0.8)/4 * 90 + 5:
            self.drive(0,200)
        self.motorLeft.stop()
        self.motorRight.stop()
        wait(20)
        self.motorLeft.reset_position()
        while self.motorLeft.position(self.rotationUnits)/5 < 550:
            self.drive(200,0)
        self.motorLeft.stop()
        self.motorRight.stop()
    def driveLab22(self):
        self.desiredDistance = 8
        while not self.hitWall():
            self.wallFollowInches(5.0)
            wait(20)
        self.motorLeft.stop()
        self.motorRight.stop()
        self.motorRight.reset_position()
        self.motorLeft.reset_position()
        wait(20)
        self.inertial.set_heading(180)
        while abs(self.inertial.heading()) > 110:
            self.drive(0,200)
            wait(20)
        self.motorLeft.stop()
        self.motorRight.stop()
        self.desiredDistance = 10
        while not self.hitWall():
            self.wallFollowInches(11.0)
            wait(20)
        self.inertial.set_heading(180)
        wait(20)
        self.motorRight.reset_position()
        self.motorLeft.reset_position()
        while abs(self.motorLeft.position(self.rotationUnits)/5) < 880:
            self.drive(-200,0)
        self.motorLeft.stop()
        self.motorRight.stop()
        while abs(self.inertial.heading()) > 110:
            self.drive(0,200)
            wait(20)
        self.inertial.set_heading(180)
        self.motorLeft.stop()
        self.motorRight.stop()
        self.motorRight.reset_position()
        self.motorLeft.reset_position()
        wait(20)
        while abs(self.motorLeft.position(self.rotationUnits)/5) < 550:
            self.drive(200,0)
        self.motorLeft.stop()
        self.motorRight.stop()
    def driveLab23(self):
        self.desiredDistance = 3
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
        while abs(self.inertial.heading()) > 110:
            self.drive(0,200)
            wait(20)
        self.drive(0,200)
        self.motorLeft.stop()
        self.motorRight.stop()
        self.desiredDistance = 8
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
        while abs(self.inertial.heading()) > 110:
            self.drive(0,200)
            wait(20)
        self.inertial.set_heading(180)
        self.motorLeft.stop()
        self.motorRight.stop()
        self.motorRight.reset_position()
        self.motorLeft.reset_position()
        wait(20)
        while abs(self.motorLeft.position(self.rotationUnits)/5) < 550:
            self.drive(200,0)
        self.motorLeft.stop()
        self.motorRight.stop()
class RobotConfig:
    def __init__(self, wheelDiameter, gearRatio, drivebaseWidth, peripherals):
        self.wheelDiameter = wheelDiameter
        self.gearRatio = gearRatio
        self.drivebaseWidth = drivebaseWidth
        self.peripherals = peripherals
    def getWheelDiameter(self):
        return self.wheelDiameter
    def getGearRatio(self):
        return self.gearRatio
    def getDrivebaseWidth(self):
        return self.drivebaseWidth
    def getCircumference(self):
        return math.pi * self.wheelDiameter
    def getDPI(self):
        return 360.0 / self.getCircumference()
    def getFrontRangeFinder(self):
        return self.peripherals[0]
    def getRightRangeFinder(self):
        return self.peripherals[1]
    def getInertialSensor(self):
        return self.peripherals[2]
    def getLeftLineSensor(self):
        return self.peripherals[3]
    def getRightLineSensor(self):
        return self.peripherals[4]
    def getBumpSwitch(self):
        return self.peripherals[5]
    def getArmMotor(self):
        return self.peripherals[6]
wheelDiameter = 4.0
wheel_travel = math.pi*wheelDiameter
track_width = 11
wheel_base = 11
gear_ratio = 5
wheelCircumference = 3.14 * wheelDiameter 
degreesPerInch = 360.0 / wheelCircumference
brain=Brain()
left_motor = Motor(Ports.PORT10, 18_1, True)
right_motor = Motor(Ports.PORT1, 18_1, False)
left_motor.set_velocity(200, RPM)
left_motor.reset_position()
right_motor.set_velocity(200, RPM)
right_motor.reset_position()
arm_motor = Motor(Ports.PORT9, 18_1, False)
arm_motor.set_velocity(200, RPM)
arm_motor.reset_position()
rangeFinderFront = Sonar(brain.three_wire_port.e)
rangeFinderRight = Sonar(brain.three_wire_port.g)
lineSensorLeft = Line(brain.three_wire_port.c)
lineSensorRight = Line(brain.three_wire_port.d)
inertial = Inertial(Ports.PORT3)
bumpSwitch = Bumper(brain.three_wire_port.a)
inertial.calibrate()
wait(2000)
peripherals = [rangeFinderFront, rangeFinderRight, inertial, lineSensorLeft, lineSensorRight, bumpSwitch, arm_motor]
robotConfig = RobotConfig(wheelDiameter, gear_ratio, track_width, peripherals)
drivebase = TankDrivebase(left_motor, right_motor, robotConfig=robotConfig, kP=7.5)
def printSensors():
    while True:
        brain.screen.print_at("arm current: ", arm_motor.current(),x=0,y=20)
        brain.screen.print_at("arm torque: ", arm_motor.torque(),x=0,y=40)
        brain.screen.print_at("arm temperature: ", arm_motor.temperature(),x=0,y=60)
        brain.screen.render()
sensorsThread = Thread(printSensors)
arm_motor.reset_position()
wait(20)
def toggleArm():
    armTo(abs(arm_motor.position()/5)-60)
def armTo(position):
    arm_motor.spin_to_position(position*5, DEGREES, 100, RPM, False)
bumpSwitch.pressed(toggleArm)
