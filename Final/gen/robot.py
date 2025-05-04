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
import math
wheelDiameter = 4.0
wheel_travel = math.pi*wheelDiameter
track_width = 11
wheel_base = 11
gear_ratio = 5
wheelCircumference = 3.14 * wheelDiameter 
degreesPerInch = 360.0 / wheelCircumference
drivePID = [0.01,0,0]
tagCameraOffset = [0, 0, 0, 0, 0, 0] #inches
objCameraOffset = [0, 0, 0, 0, 0, 0] #inches
vision_orange = Colordesc(1,245,134,89,5,0.17)
vision_yellow = Colordesc(1,245,134,89,5,0.17)
vision_green = Colordesc(1,245,134,89,5,0.17)
tagCameraWidth = 320
tagCameraHeight = 240
tagCameraXOffset = tagCameraWidth/2
tagCameraYOffset = tagCameraHeight/2
objCameraWidth = 320
objCameraHeight = 240
objCameraXOffset = objCameraWidth/2
objCameraYOffset = objCameraHeight/2
fruitHeight1 = 5 #inches
fruitHeight2 = 10 #inches
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
rollerDiameter = 2.5 #inches
rollerGearRatio = 5
forksGearRatio = 5
lowFruitHeight = 5 #inches
lowFruitAngle = 45 #degrees
highFruitHeight = 10 #inches
highFruitAngle = 90 #degrees
lowFruitWristAngle = 5 #degrees
highFruitWristAngle = 10 #degrees
minArmLength = 0
maxArmLength = 270
minArmAngle = 1
maxArmAngle = 90
minWristAngle = 2
maxWristAngle = 180
armGearRatio = 12/6
pivotGearRatio = 72/12
wristGearRatio = 10/6
armPID = [1.5,0,0]
armFF = 0.5
armTolerance = 0.5
pivotPID = [1.8,0,0]
pivotFF = 0.5
pivotTolerance = 0.1
wristPID = [1,0,0]
wristTolerance = 1
post1Height = [0, 25, 20]
post2Height = [0, 45, 45]
post3Height = [45, 70, 70]
post4Height = [270, 55, 55]
class TankDrivebase ():
    diameter = 4
    gearing = 5 / 1
    wheelBase = 11
    circumference = math.pi * diameter
    def __init__(self, _motorLeft: Motor, _motorRight: Motor, gyro: Inertial, tagCamera: AiVision, camera: Vision, ultraSonic: Sonar, 
                 motorCorrectionConfig = None, _rotationUnits = DEGREES, _speedUnits = RPM):
        self.motorLeft = _motorLeft
        self.motorRight = _motorRight
        self.ultraSonic = ultraSonic
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
        odometryThread = Thread(self.updateOdometry)
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
        pass
    def onLine(self, sensor):
        whiteLineValue = 680
        return sensor.reflectivity() < whiteLineValue
    def drive(self, speed, direction):
        left_speed = speed - direction
        right_speed = speed + direction
        self.motorLeft.spin(FORWARD,left_speed)
        self.motorRight.spin(FORWARD,right_speed)
    def goUpRamp(self):
        self.motorLeft.spin(FORWARD, -50)
        self.motorRight.spin(FORWARD, -50)
        while self.gyro.orientation(OrientationType.ROLL) < 0:
            rightError = self.ultraSonic.distance(INCHES) - 3.2
            self.drive(-100, -drivePID[0]*rightError)
            wait(20)
        self.motorLeft.stop()
        self.motorRight.stop()
class Arm ():
    def __init__(self, armmotorL: Motor, armmotorR: Motor, pivotmotorL: Motor, pivotmotorR: Motor, wristmotor: Motor):
        self.armmotorL = armmotorL
        self.armmotorR = armmotorR
        self.pivotmotorL = pivotmotorL
        self.pivotmotorR = pivotmotorR
        self.wristmotor = wristmotor
        self.armLength = 0
        self.armAngle = 0
        self.wristAngle = 0
        self.dLength = 0
        self.dAngle = 0
        self.dWrist = 0
        armLengthThread = Thread(lambda: self.setLength())
        armAngleThread = Thread(lambda: self.setAngle())
        wristAngleThread = Thread(lambda: self.setWristAngle())
    def getLength(self):
        self.armLength = self.armmotorL.position(DEGREES) / armGearRatio
        return self.armLength
    def setLength(self):
        while True:
            targetLength = self.clamp(self.dLength, minArmLength, maxArmLength)
            kp, ki, kd = armPID
            integral = 0
            prevError = 0
            currentLength = self.getLength()
            error = targetLength - currentLength
            integral += error
            derivative = error - prevError
            output = kp * error + ki * integral + kd * derivative
            self.armmotorL.spin(FORWARD, output, PERCENT)
            self.armmotorR.spin(FORWARD, output, PERCENT)
            prevError = error
            self.armLength = self.armmotorL.position(DEGREES) / armGearRatio
    def getAngle(self):
        self.armAngle = self.pivotmotorL.position(DEGREES) / pivotGearRatio
        return self.armAngle
    def setAngle(self):
        while True:
            targetAngle = self.clamp(self.dAngle, minArmAngle, maxArmAngle)
            kp, ki, kd = pivotPID
            integral = 0
            prevError = 0
            currentAngle = self.getAngle()
            error = targetAngle - currentAngle
            integral += error
            derivative = error - prevError
            output = (kp * error + pivotFF*math.cos(math.radians(self.getAngle()))) + ki * integral + kd * derivative
            self.pivotmotorL.spin(FORWARD, output, PERCENT)
            self.pivotmotorR.spin(FORWARD, output, PERCENT)
            prevError = error
            self.armAngle = self.pivotmotorL.position(DEGREES) / pivotGearRatio
    def getWristAngle(self):
        self.wristAngle = self.wristmotor.position(DEGREES) / wristGearRatio
        return self.wristmotor.position(DEGREES) / wristGearRatio
    def setWristAngle(self):
        while True:
            targetAngle = self.clamp(self.dWrist, minWristAngle, maxWristAngle)
            kp, ki, kd = wristPID
            integral = 0
            prevError = 0
            currentAngle = self.getWristAngle()
            error = targetAngle - currentAngle
            integral += error
            derivative = error - prevError
            output = kp * error + ki * integral + kd * derivative
            self.wristmotor.spin(FORWARD, output, PERCENT)
            prevError = error
            self.wristAngle = self.wristmotor.position(DEGREES) / wristGearRatio
    def clamp(self, value, min_value, max_value):
        return max(min_value, min(value, max_value))
    def setSetpoint(self, length, angle, wristAngle):
        self.dLength = length
        self.dAngle = angle
        self.dWrist = wristAngle
class Intake ():
    def __init__(self, intakemotor: Motor):
        self.intakemotor = intakemotor
        self.intakemotor.set_velocity(100, PERCENT)
        self.escapeLoop = False
    def runIntake(self, direction):
        if direction == FORWARD:
            self.intakemotor.spin(FORWARD)
        else:
            self.intakemotor.spin(REVERSE)
    def stopIntake(self):
        self.escapeLoop = True
        self.intakemotor.stop()
        self.escapeLoop = False
    def intakeUntilCurrent(self):
        while self.intakemotor.current() < 0.5 or self.escapeLoop == False:
            self.runIntake("reverse")
        self.stopIntake()
def setSubsystems(driveBase: TankDrivebase, arm: Arm, intake: Intake):
    global driveSub, armSub, intakeSub, forksSub
    driveSub = driveBase
    armSub = arm
    intakeSub = intake
def stowArm():
    intakeSub.stopIntake()
    armSub.setSetpoint(minArmLength, minArmAngle, minWristAngle)
brain=Brain()
motorLeft = Motor(Ports.PORT20, 18_1, False)
motorRight = Motor(Ports.PORT10, 18_1, True)
sideUltrasonic = Sonar(brain.three_wire_port.e)
arm_motorL = Motor(Ports.PORT17, 18_1, False)
arm_motorR = Motor(Ports.PORT7, 18_1, True)
intake_motor = Motor(Ports.PORT6, 18_1, False)
wrist_motor = Motor(Ports.PORT16, 18_1, False)
pivot_motorL = Motor(Ports.PORT18, 18_1, True)
pivot_motorR = Motor(Ports.PORT9, 18_1, False)
inertial = Inertial(Ports.PORT1)
inertial.set_heading(180, DEGREES)
inertial.calibrate()
camera = Vision(Ports.PORT6, 50)
tagCamera = AiVision(Ports.PORT2, 8)
tagCamera.start_awb()
controller = Controller()
drivebase = TankDrivebase(motorLeft, motorRight, inertial, tagCamera, camera, sideUltrasonic)
intake = Intake(intake_motor)
arm = Arm(arm_motorL, arm_motorR, pivot_motorL, pivot_motorR, wrist_motor)
setSubsystems(drivebase, arm, intake)
def printDebugging():
    while True:
        brain.screen.print_at(arm.getLength(), x=1, y=60)
        brain.screen.print_at(arm.getAngle(), x=1, y=80)
        brain.screen.print_at(arm.getWristAngle(), x=1, y=100)
        sleep(20)
debugThread = Thread(printDebugging)
while inertial.is_calibrating():
    sleep(10)
arm.setSetpoint(270, 46, 46)
intake.runIntake(FORWARD)
