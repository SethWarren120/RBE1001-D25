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
track_width = 15
wheel_base = 9.5
gear_ratio = 1
wheelCircumference = 3.14 * wheelDiameter 
degreesPerInch = 360.0 / wheelCircumference
drivePID = [0.01,0,0]
tagCameraOffset = [0, 0, 0, 0, 0, 0] #inches
objCameraOffset = [0, 0, 0, 0, 0, 0] #inches
vision_Yellow = Colordesc(2, 140, 106, 60, 7, 0.37)
vision_Green = Colordesc(3, 14, 143, 61, 15, 0.41)
vision_Orange = Colordesc(1, 232, 152, 138, 10, 0.12)
vision_Pink = Colordesc(3, 232, 78, 146, 11, 0.13)
vision_GreenBox = Codedesc(1, vision_Pink, vision_Green)
vision_YellowBox = Codedesc(2, vision_Pink, vision_Yellow)
vision_OrangeBox = Codedesc(3, vision_Pink, vision_Orange)
cameraWidth = 320
cameraHeight = 240
cameraXOffset = cameraWidth/2
cameraYOffset = cameraHeight/2
cameraFOV = 61 #degrees
cameraVerticalFOV = 41 #degrees
visionFruitSizeThreshold = 30
visionContinuityTolerance = 5
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
minArmLength = 0
maxArmLength = 270
minArmAngle = 1
maxArmAngle = 90
armGearRatio = 12/6
pivotGearRatio = 72/12
wristGearRatio = 10/6
armPID = [1.5,0,0]
armFF = 0.1
armTolerance = 0.5
pivotPID = [1,0,0]
pivotFF = 0.09
pivotTolerance = 0.2
pivotMaxSpeed = 15
wristPID = [1,0,0]
wristTolerance = 0.5
post1Height = [180, 25, 20]
post2Height = [180, 50, 50]
post3Height = [140, 42, 42]
post4Height = [270, 52, 57]
fruitAlignmentPID = [0.1, 0, 0]
fruitAlignmentTolerance = 10
Lines = [[0, 0, 36, 0], [36, 0, 36, 60], [36, 60, 0, 60], [0, 60, 0, 0], 
         [36, 20, 24, 20], [36, 40, 24, 40], [0, 40, 12, 40], [0, 20, 12, 20]]
LineConnections = [[-1, [36, 0], -1, [0, 0], -1, -1, -1, -1],
                   [[36, 0], -1, [36, 60], -1, [36, 20], [36, 40], -1, -1],
                   [-1, [36, 60], -1, [0, 60], -1, -1, -1, -1],
                   [[0, 0], -1, [0, 60], -1, -1, -1, [0, 40], [0, 20]],
                   [-1, [36, 20], -1, -1, -1, -1, -1, -1],
                   [-1, [36, 40], -1, -1, -1, -1, -1, -1],
                   [-1, -1, -1, [0, 40], -1, -1, -1, -1],
                   [-1, -1, -1, [0, 20], -1, -1, -1, -1]]
LineNav = [[[0], [1], [1, 2], [3], [1, 4], [1, 5], [3, 6], [3, 7]],
           [[0], [1], [2], [0, 3], [4], [5], [2, 3, 6], [0, 3, 7]],
           [[3, 0], [1], [2], [3], [1, 4], [1, 5], [3, 6], [3, 7]],
           [[0], [0, 1], [2], [3], [0, 1, 4], [2, 1, 5], [6], [7]],
           [[1, 0], [1], [1, 2], [1, 0, 3], [4], [1, 5], [1, 0, 3, 6], [1, 0, 3, 7]],
           [[1, 0], [1], [1, 2], [1, 2, 3], [1, 4], [5], [1, 2, 3, 6], [1, 2, 3, 7]],
           [[3, 0], [3, 2, 1], [3, 2], [3], [3, 2, 1, 4], [3, 2, 1, 5], [6], [3, 7]],
           [[3, 0], [3, 0, 1], [3, 2], [3], [3, 0, 1, 4], [3, 0, 1, 5], [3, 6], [7]]]
class TankDrivebase ():
    diameter = 4
    gearing = 5 / 1
    wheelBase = 11
    circumference = math.pi * diameter
    def __init__(self, _motorLeft: Motor, _motorRight: Motor, gyro: Inertial, camera: AiVision, ultraSonic: Sonar, 
                 motorCorrectionConfig = None, _rotationUnits = DEGREES, _speedUnits = RPM):
        self.motorLeft = _motorLeft
        self.motorRight = _motorRight
        self.ultraSonic = ultraSonic
        self.camera = camera
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
    def drive(self, speed, direction):
        left_speed = speed - direction
        right_speed = speed + direction
        self.motorLeft.spin(FORWARD,left_speed)
        self.motorRight.spin(FORWARD,right_speed)
    def turn(self, heading):
        pass
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
    def onLine(self, sensor):
        whiteLineValue = 680
        return sensor.reflectivity() < whiteLineValue
    def centerToObject(self):
        while True:
            object = None
            while True:
                objectsGreen = self.camera.take_snapshot(vision_Green)
                objectsOrange = self.camera.take_snapshot(vision_Orange)
                objectsYellow = self.camera.take_snapshot(vision_Yellow)
                print(objectsOrange)
                largestwidth = 0
                for tobject in objectsGreen:
                    if tobject.width > largestwidth:
                        largestwidth = tobject.width
                        object = tobject
                        print("largest is green")
                for tobject in objectsOrange:
                    if tobject.width > largestwidth:
                        largestwidth = tobject.width
                        object = tobject
                        print("largest is orange")
                for tobject in objectsYellow:
                    if tobject.width > largestwidth:
                        largestwidth = tobject.width
                        object = tobject
                        print("largest is yellow")
                if object != None:
                    break
            x = object.centerX
            correctedX = x + cameraXOffset
            print("turning")
            self.turn(correctedX)
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
        leftLength = self.armmotorL.position(DEGREES) / armGearRatio
        rightLength = self.armmotorR.position(DEGREES) / armGearRatio
        self.armLength = (leftLength+rightLength)/2
        return self.armLength
    def setLength(self):
        while True:
            targetLength = self.clamp(self.dLength, minArmLength, maxArmLength)
            if (abs(self.getAngle()-self.dAngle) > pivotTolerance*5):
                targetLength = self.getLength()
            kp, ki, kd = armPID
            integral = 0
            prevError = 0
            currentLength = self.getLength()
            error = targetLength - currentLength
            integral += error
            derivative = error - prevError
            output = kp * error + ki * integral + kd * derivative
            if (self.armmotorL.torque() > 1.5 or self.armmotorL.torque() > 1.5 or (self.getLength() == 0 and targetLength == 0)):
                self.armmotorR.stop()
                self.armmotorL.stop()
                self.armmotorR.set_position(0, DEGREES)
                self.armmotorL.set_position(0, DEGREES)
            else:
                self.armmotorL.spin(FORWARD, output, PERCENT)
                self.armmotorR.spin(FORWARD, output, PERCENT)
            prevError = error
            self.armLength = self.armmotorL.position(DEGREES) / armGearRatio
    def getAngle(self):
        leftAngle = self.pivotmotorL.position(DEGREES) / pivotGearRatio
        rightAngle = self.pivotmotorR.position(DEGREES) / pivotGearRatio
        self.armAngle = (leftAngle+rightAngle)/2
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
            clampedOutput = self.clamp(output, -pivotMaxSpeed, pivotMaxSpeed)
            if (self.pivotmotorL.torque() > 1.5 or self.pivotmotorR.torque() > 1.5 or (self.getAngle() == 0 and targetAngle == 0)):
                self.pivotmotorR.stop()
                self.pivotmotorL.stop()
                self.pivotmotorR.set_position(0, DEGREES)
                self.pivotmotorL.set_position(0, DEGREES)
            else:
                self.pivotmotorL.spin(FORWARD, clampedOutput, PERCENT)
                self.pivotmotorR.spin(FORWARD, clampedOutput, PERCENT)
            prevError = error
            self.armAngle = self.pivotmotorL.position(DEGREES) / pivotGearRatio
    def getWristAngle(self):
        self.wristAngle = self.wristmotor.position(DEGREES) / wristGearRatio
        return self.wristAngle
    def setWristAngle(self):
        while True:
            targetAngle = self.dWrist
            if targetAngle > 180:
                targetAngle -= 360
            if (abs(self.getLength()-self.dLength) > armTolerance*10 and self.dLength > 130):
                targetAngle = self.getWristAngle()
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
    def flipWrist(self):
        self.setSetpoint(self.dLength, self.dAngle, self.dWrist+180)
    def clamp(self, value, min_value, max_value):
        return max(min_value, min(value, max_value))
    def setSetpoint(self, length, angle, wristAngle):
        self.dAngle = angle
        self.dLength = length
        self.dWrist = wristAngle
    def stowArm(self):
        currentWristAngle = self.getWristAngle()
        print(currentWristAngle)
        if abs(currentWristAngle - 0) < abs(currentWristAngle - 180):
            if abs(currentWristAngle - 0) < abs(currentWristAngle - 180) and abs(currentWristAngle - 0) < abs(currentWristAngle + 180):
                self.setSetpoint(self.getLength(), self.getAngle(), 0)
            elif abs(currentWristAngle - 180) < abs(currentWristAngle + 180):
                self.setSetpoint(self.getLength(), self.getAngle(), 180)
            else:
                self.setSetpoint(self.getLength(), self.getAngle(), -180)
        while(abs(self.getWristAngle()-self.dWrist) > wristTolerance):
            sleep(1)
        self.setSetpoint(0, self.getAngle(), self.getWristAngle())
        while(abs(self.getLength()-self.dLength) > armTolerance):
            sleep(1)
        self.setSetpoint(0, 0, self.getWristAngle())
    def atSetpoint(self):
        if (abs(self.getLength()-self.dLength) < armTolerance and abs(self.getAngle()-self.dAngle) < pivotTolerance and abs(self.getWristAngle()-self.dWrist) < wristTolerance):
            return True
        else:
            return False
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
    def runTimeThread(self, direction, time):
        self.runIntake(direction)
        sleep(time)
        self.stopIntake()
    def runIntakeForTime(self, direction, time):
        intakeTimeThread = Thread(lambda: self.runTimeThread(direction, time))
def setSubsystems(driveBase: TankDrivebase, arm: Arm, intake: Intake):
    global driveSub, armSub, intakeSub, forksSub
    driveSub = driveBase
    armSub = arm
    intakeSub = intake
tracking_enabled = False
def startObjectTracking():
    global tracking_enabled
    tracking_enabled = True
    tracking_thread = Thread(continuousArmTracking)
def stopObjectTracking():
    global tracking_enabled
    tracking_enabled = False
def continuousArmTracking():
    global tracking_enabled
    while tracking_enabled:
        findAndAimAtObject()
        wait(50)
def findAndAimAtObject():
    object = None
    objectsGreen = driveSub.camera.take_snapshot(vision_Green)
    objectsOrange = driveSub.camera.take_snapshot(vision_Orange)
    objectsYellow = driveSub.camera.take_snapshot(vision_Yellow)
    largestwidth = 0
    for tobject in objectsGreen:
        if tobject.width > largestwidth:
            largestwidth = tobject.width
            object = tobject
    for tobject in objectsOrange:
        if tobject.width > largestwidth:
            largestwidth = tobject.width
            object = tobject
    for tobject in objectsYellow:
        if tobject.width > largestwidth:
            largestwidth = tobject.width
            object = tobject
    if object is None:
        return
    y = object.centerY
    correctedY = cameraYOffset - y
    angleAdjustment = correctedY * (pivotPID[0] / (cameraHeight / 2))
    currentAngle = armSub.getAngle()
    targetAngle = currentAngle + angleAdjustment
    armSub.setSetpoint(
        armSub.getLength(),
        targetAngle,
        armSub.getWristAngle()
    )
def angleArmToObject():
    startObjectTracking()
def grabObject():
    armSub.setSetpoint(0, 50, 0)
    wait(1000)
    armSub.setSetpoint(50, 50, 0)
    wait(1000)
    armSub.setSetpoint(50, 50, armSub.getAngle())
    intakeSub.runIntakeForTime(FORWARD, 1000)
    armSub.stowArm()
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
camera = AiVision(Ports.PORT2, vision_Yellow, vision_Green, vision_Orange, vision_Pink, vision_GreenBox, vision_YellowBox, vision_OrangeBox, AiVision.ALL_TAGS)
camera.start_awb()
controller = Controller()
drivebase = TankDrivebase(motorLeft, motorRight, inertial, camera, sideUltrasonic)
intake = Intake(intake_motor)
arm = Arm(arm_motorL, arm_motorR, pivot_motorL, pivot_motorR, wrist_motor)
setSubsystems(drivebase, arm, intake)
def printDebugging():
    while True:
        brain.screen.print_at(arm.getLength(), x=1, y=60)
        brain.screen.print_at(arm.getAngle(), x=1, y=80)
        brain.screen.print_at(arm.getWristAngle(), x=1, y=100)
        brain.screen.print_at(arm.pivotmotorR.torque(), x=1, y=120)
        sleep(20)
debugThread = Thread(printDebugging)
while inertial.is_calibrating():
    sleep(1)
arm.setSetpoint(post3Height[0],post3Height[1]+15,-90)
sleep(4000)
arm.setSetpoint(post3Height[0],post3Height[1]+15,post3Height[2]+180)
sleep(1000)
intake.runIntakeForTime(FORWARD,2000)
sleep(3000)
arm.stowArm()
