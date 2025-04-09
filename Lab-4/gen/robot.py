# Automatically generated deployment script
# Edits to this file will be overwritten
from vex import *
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
vision_orange = Colordesc(1,245,134,89,5,0.17)
cameraOffset = [0, 0, 0] #inches
cameraFOV = 75 #degrees
cameraXOffset = -320/2
cameraYOffset = -240/2
fruitHeight1 = 5 #inches
fruitHeight2 = 10 #inches
smallFruitWidth = 2 + 3/8 #inches
largeFruitWidth = 5.5 #inches
objectThreashold = 5
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
class Subsystem():
    def __init__(self):
        self.defaultCommand = None
        self.currentCommand = None
        self.periodic()
    def setDefaultCommand(self, command: function):
        self.defaultCommand = command
    def run(self, command):
        self.currentCommand = command
        runThread = Thread(command)
    def periodic(self):
        while True:
            if (self.currentCommand == None and self.defaultCommand != None):
                self.run(self.defaultCommand)
            wait(20)
class TankDrivebase (Subsystem):
    def __init__(self, _motorLeft, _motorRight, gyro: Inertial, vision: AiVision,
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
    def moveLen(self, len, speed):
        deg = 360 * ((len / wheelCircumference) * gear_ratio)
        self.motorCorrector.setPassiveMode(False)
        self.motorCorrector.correctMotors([deg, deg])
        self.motorLeft.spin_for(FORWARD, deg, self.rotationUnits, speed, self.speedUnits, False)
        self.motorRight.spin_for(FORWARD, deg, self.rotationUnits, speed, self.speedUnits, True)
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
    def driveCommand(self, leftAxis, rightAxis):
        self.motorLeft.spin(FORWARD, leftAxis + rightAxis, self.speedUnits)
        self.motorRight.spin(FORWARD, leftAxis - rightAxis, self.speedUnits)
    def centerToObject(self):
        objects = None
        while True:
            objects = self.vision.take_snapshot(vision_orange)
            if len(objects) > 0:
                break
        x = objects[0].centerX
        y = objects[0].centerY
        correctedX = x + cameraXOffset
        correctedY = y + cameraYOffset
        self.motorLeft.spin(FORWARD, correctedX*0.8, self.speedUnits)
        self.motorRight.spin(FORWARD, -correctedX*0.8, self.speedUnits)
    def stayDistanceFromObject(self):
        objects = None
        while True:
            objects = self.vision.take_snapshot(vision_orange)
            if len(objects) > 0:
                break
        viewedWidth = objects[0].width
        widthChange = viewedWidth - 68
        self.motorLeft.spin(FORWARD, (widthChange)*0.8, self.speedUnits)
        self.motorRight.spin(FORWARD, (widthChange)*0.8, self.speedUnits)
    def lab(self):
        while True:
            self.centerToObject()
            self.stayDistanceFromObject()
            wait(20)
def setSubsystems(driveBase: TankDrivebase):
    global driveSub
    driveSub = driveBase
brain=Brain()
l_motor = Motor(Ports.PORT1, 18_1, True)
r_motor = Motor(Ports.PORT10, 18_1, False)
inertial = Inertial(Ports.PORT3)
camera = AiVision(Ports.PORT9, vision_orange)
drivebase = TankDrivebase(l_motor, r_motor, inertial, camera)
controller = Controller()
setSubsystems(drivebase)
def visionFunc():
    while True:
        brain.screen.print_at("running", x=0, y=80)
        objects = camera.take_snapshot(vision_orange)
        if len(objects) > 0:
            brain.screen.print_at("Object found", x=0, y=20)
            brain.screen.print_at("X: " + str(objects[0].centerX+cameraXOffset), x=0, y=40)
            brain.screen.print_at("Y: " + str(objects[0].centerY+cameraYOffset), x=0, y=60)
            brain.screen.render()
        wait(20)
drivebase.lab()
