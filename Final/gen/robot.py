# Automatically generated deployment script
# Edits to this file will be overwritten
from vex import *
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
vision_Green = Colordesc(1, 14, 143, 61, 15, 0.41)
vision_Yellow = Colordesc(2, 140, 106, 60, 7, 0.37)
vision_Orange = Colordesc(3, 232, 152, 138, 10, 0.12)
vision_Pink = Colordesc(4, 232, 78, 146, 11, 0.13)
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
visionContinuityTolerance = 30
fruitHeight1 = 5 #inches
fruitHeight2 = 10 #inches
tagLocations = [[13, -0.75, 90],
                [86, -0.75, 90],
                [98.75, 13, 180],
                [98.75, 50, 180],
                [98.75, 90, 180],
                [98.75, 133, 180],
                [86, 146.75, 270],
                [13, 146.75, 270],
                [-0.75, 133, 0],
                [-0.75, 90, 0],
                [-0.75, 50, 0],
                [-0.75, 13, 0]]
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
post1Height = [160, 15, 10]
post2Height = [140, 40, 45]
post3Height = [180, 50, 42]
post4Height = [150, 79, 75]
fruitAlignmentPID = [0.1, 0, 0]
fruitAlignmentTolerance = 10
Lines = [[13, 13, 86, 13], [86, 13, 86, 133], [86, 133, 13, 133], [13, 133, 13, 13], 
         [86, 50, 24, 50], [86, 90, 24, 90], [13, 90, 12, 90], [13, 50, 12, 50]]
LineConnections = [[-1, [86, 13], -1, [13, 13], -1, -1, -1, -1],
                   [[86, 13], -1, [86, 133], -1, [86, 50], [86, 90], -1, -1],
                   [-1, [86, 133], -1, [13, 133], -1, -1, -1, -1],
                   [[13, 13], -1, [13, 133], -1, -1, -1, [13, 90], [13, 50]],
                   [-1, [86, 50], -1, -1, -1, -1, -1, -1],
                   [-1, [86, 90], -1, -1, -1, -1, -1, -1],
                   [-1, -1, -1, [13, 90], -1, -1, -1, -1],
                   [-1, -1, -1, [13, 50], -1, -1, -1, -1]]
LineNav = [[[0], [0,1], [0,1, 2], [0,3], [0,1, 4], [0,1, 5], [0,3, 6], [0,3, 7]],
           [[1,0], [1], [1,2], [1,0, 3], [1,4], [1,5], [1,2, 3, 6], [1,0, 3, 7]],
           [[2,3, 0], [2,1], [2], [2,3], [2,1, 4], [2,1, 5], [2,3, 6], [2,3, 7]],
           [[3,0], [3,0, 1], [3,2], [3], [3,0, 1, 4], [3,2, 1, 5], [3,6], [3,7]],
           [[4,1, 0], [4,1], [4,1, 2], [4,1, 0, 3], [4], [4,1, 5], [4,1, 0, 3, 6], [4,1, 0, 3, 7]],
           [[5,1, 0], [5,1], [5,1, 2], [5,1, 2, 3], [5,1, 4], [5], [5,1, 2, 3, 6], [5,1, 2, 3, 7]],
           [[6,3, 0], [6,3, 2, 1], [6,3, 2], [6,3], [6,3, 2, 1, 4], [6,3, 2, 1, 5], [6], [6,3, 7]],
           [[7,3, 0], [7,3, 0, 1], [7,3, 2], [7,3], [7,3, 0, 1, 4], [7,3, 0, 1, 5], [7,3, 6], [7]]]
ultrasonicWallClearance = 3.5
ultrasonicWallFollowPID = [5, 0, 0]
headingSetpointTolerance = 3
headingSetpointPID = [0.6, 0, 0]
fieldSetpointTolerance = 2
fieldDriveHeadingPID = [1, 0, 0]
fieldDriveSpeed = 30
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
fieldX = 74
fieldY = 13
wheelEstimates = [[0.0, 0.0],
                  [0.0, 0.0],
                  [0.0, 0.0],
                  [0.0, 0.0],
                  [0.0, 0.0]]
inertial = Inertial(Ports.PORT1)
def odometryEstimate(wheelL, wheelR):
    global fieldX, fieldY
    if wheelL == 0:
        wheelL = 0.0001
    if wheelR == 0:
        wheelR = 0.0001
    if wheelL < wheelR:
        ri = 15.0 * wheelL / (wheelR - wheelL)
        theta = 2.0 * math.pi * wheelL / ri
        d = (ri + 7.5) * theta / (2 * math.pi)
    elif wheelR < wheelL:
        ri = 15.0 * wheelR / (wheelL - wheelR)
        theta = -(2.0 * math.pi * wheelR / ri)
        d = (ri + 7.5) * -theta / (2 * math.pi)
    else:
        d = wheelL
        theta = 0
    heading = ((360 - inertial.orientation(YAW)) * math.pi / 180.0)
    x = d * math.cos(heading)
    y = d * math.sin(heading)
    wheelEstimates.remove(wheelEstimates[0])
    wheelEstimates.append([x, y])
    fieldX += x
    fieldY += y
def visionEstimate(x, y):
    global fieldX, fieldY
    fieldX = x
    fieldY = y
    for odomEstimate in wheelEstimates:
        fieldX += odomEstimate[0]
        fieldY += odomEstimate[1]
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
    def drive(self, speed, direction):
        left_speed = speed - direction
        right_speed = speed + direction
        self.motorLeft.spin(FORWARD,left_speed)
        self.motorRight.spin(FORWARD,right_speed)
    def headingError(self, setpoint, current):
        err = setpoint - current
        while err < -180:
            err += 360
        while err > 180:
            err -= 360
        return err
    def turn(self, heading):        
        print(360 - inertial.orientation(YAW))
        print(heading)
        while abs(self.headingError(heading, 360-inertial.orientation(YAW))) > headingSetpointTolerance:
            print(self.headingError(heading, 360-inertial.orientation(YAW)))
            self.drive(0, headingSetpointPID[0] * 360-self.headingError(heading, inertial.orientation(YAW)))
        print("Done")
        self.drive(0, 0)
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
        wheelL = self.motorLeft.position(DEGREES)
        wheelR = self.motorRight.position(DEGREES)
        while True:
            dl = math.pi * (self.motorLeft.position(DEGREES) - wheelL) / 90
            dr = math.pi * (self.motorRight.position(DEGREES) - wheelR) / 90
            wheelL = self.motorLeft.position(DEGREES)
            wheelR = self.motorRight.position(DEGREES)
            odometryEstimate(dl, dr)
            sleep(20)
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
            wait(20)
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
            wait(20)
    def resetAngle(self):
        print("qeegiuadfikuaswefhuWE;UHHAER")
        while True:
            output = -20
            self.pivotmotorL.spin(FORWARD, output, PERCENT)
            self.pivotmotorR.spin(FORWARD, output, PERCENT)
            if (self.pivotmotorL.torque() > 1.5 or self.pivotmotorR.torque() > 1.5):
                self.pivotmotorR.stop()
                self.pivotmotorL.stop()
                self.pivotmotorR.set_position(0, DEGREES)
                self.pivotmotorL.set_position(0, DEGREES)
                break
            wait(20)
        print("done 1")
        while True:
            targetLength = -1000
            output = -3
            self.armmotorL.spin(FORWARD, output, PERCENT)
            self.armmotorR.spin(FORWARD, output, PERCENT)
            if (self.armmotorL.torque() > 1.5 or self.armmotorL.torque() > 1.5):
                self.armmotorR.stop()
                self.armmotorL.stop()
                self.armmotorR.set_position(0, DEGREES)
                self.armmotorL.set_position(0, DEGREES)
                break
        print("done")
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
            wait(20)
    def flipWrist(self):
        self.setSetpoint(self.dLength, self.dAngle, self.dWrist+180)
    def clamp(self, value, min_value, max_value):
        return max(min_value, min(value, max_value))
    def setSetpoint(self, length, angle, wristAngle):
        self.dAngle = angle
        self.dLength = length
        self.dWrist = wristAngle
    def stowArm(self):
        self.dWrist = 0
        while(abs(self.getWristAngle()-self.dWrist) > wristTolerance):
            sleep(1)
        self.dLength = 0
        while(abs(self.getLength()-self.dLength) > armTolerance):
            sleep(1)
        self.dAngle = 0
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
treeLocations = [
    1, 2,
    1, 2,
    1, 2,
    1, 2,
    1, 2,
    1, 2,
    1, 2,
    1, 2,
    1, 2
]
treeHeightIntervals = [0, 3, 6, 9, 12, 15, 18, 21]
treeHeights = [-1, -1, -1, -1, -1, -1, -1, -1, -1]
treeHeightSums = [-1, -1, -1, -1, -1, -1, -1, -1, -1]
treeHeightCounts = [-1, -1, -1, -1, -1, -1, -1, -1, -1]
treeRotations = [0, 0, 0, 0, 0, 0, 0, 0, 0]
treeRotationCertainty = [-1, -1, -1, -1, -1, -1, -1, -1, -1]
fruitStatus = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
             0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
def GetTreePosition(treeNum):
    return { treeLocations[2*treeNum], treeLocations[2*treeNum + 1] }
def DetermineNextFruit(currentTree):
    pass
def GiveTreeHeightEstimate(tree, height, certainty):
    treeHeightSums[tree] += height
    treeHeightCounts[tree] += certainty
    bestHeight = 0
    i = 1
    while i < 8:
        if abs((treeHeightSums[tree] / treeHeightCounts[tree]) - treeHeightIntervals[i]) < abs((treeHeightSums[tree] / treeHeightCounts[tree]) - treeHeightIntervals[bestHeight]):
            bestHeight = i
    treeHeights[tree] = bestHeight
def GiveTreeRotationEstimate(tree, rotation, certainty):
    if certainty >= treeRotationCertainty[tree]:
        treeRotations[tree] = rotation
        treeRotationCertainty[tree] = certainty
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
def dumpObject():
    currentWristAngle = armSub.getWristAngle()
    if currentWristAngle < 180 and currentWristAngle > 0:
        armSub.setSetpoint(140, 20, 90)
    elif currentWristAngle > 180:
        armSub.setSetpoint(140, 20, 270)
    else:
        armSub.setSetpoint(140, 20, -90)
    sleep(3000)
    intakeSub.runIntakeForTime(REVERSE, 2000)
    sleep(3000)
    armSub.stowArm()
cam = AiVision(Ports.PORT2, AiVision.ALL_TAGS)
cam.take_snapshot(AiVision.ALL_TAGS, 8)
def AprilTagLoop():
    while True:
        tags = cam.take_snapshot(AiVision.ALL_TAGS)
        print(len(tags))
        for tag in tags:
            print(tag.id)
            ProcessAprilTag(tag)
        sleep(20)
def ProcessAprilTag(aprilTag):
    if aprilTag.id < 12:
        distance = 290 / aprilTag.height
        angle = 27.5 - (27.5 * (aprilTag.centerY)/120)
        height = math.sin(angle * 2 * math.pi / 360) * distance
        GiveTreeHeightEstimate(aprilTag.id, height, aprilTag.height)
        cr = 0 #Current rotation, get from gyro
        GiveTreeRotationEstimate(aprilTag.id, cr + aprilTag.rotation, aprilTag.height)
    else:
        distance = 730 / aprilTag.height
        fieldAngle = math.pi * (360 - (aprilTag.angle - tagLocations[aprilTag.id - 12][2])) / 180
        fieldx = tagLocations[aprilTag.id - 12][0] + distance * math.cos(fieldAngle)
        fieldy = tagLocations[aprilTag.id - 12][1] + distance * math.sin(fieldAngle)
        print([fieldx, fieldy])
        visionEstimate(fieldx, fieldy)
brain=Brain()
sleep(2000)
motorLeft = Motor(Ports.PORT20, 18_1, False)
motorRight = Motor(Ports.PORT10, 18_1, True)
sideUltrasonic = Sonar(brain.three_wire_port.e)
inertial = Inertial(Ports.PORT1)
arm_motorL = Motor(Ports.PORT17, 18_1, False)
arm_motorR = Motor(Ports.PORT7, 18_1, True)
intake_motor = Motor(Ports.PORT5, 18_1, False)
wrist_motor = Motor(Ports.PORT3, 18_1, False)
pivot_motorL = Motor(Ports.PORT4, 18_1, True)
pivot_motorR = Motor(Ports.PORT9, 18_1, False)
camera = AiVision(Ports.PORT2, vision_Green, vision_Yellow, vision_Orange, vision_Pink, vision_GreenBox, vision_YellowBox, vision_OrangeBox, AiVision.ALL_TAGS)
camera.start_awb()
controller = Controller()
drivebase = TankDrivebase(motorLeft, motorRight, inertial, camera, sideUltrasonic, controller)
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
def grabFruit():
    intake.runIntake(FORWARD)
    arm.setSetpoint(arm.dLength, arm.dAngle, arm.dWrist-5)
armLengthThread = Thread(lambda: arm.setLength())
armAngleThread = Thread(lambda: arm.setAngle())
wristAngleThread = Thread(lambda: arm.setWristAngle())
controller.buttonR2.pressed(grabFruit)
controller.buttonR2.released(intake.stopIntake)
controller.buttonL2.pressed(arm.flipWrist)
controller.buttonB.pressed(dumpObject)
controller.buttonUp.pressed(lambda: arm.setSetpoint(post4Height[0], post4Height[1], post4Height[2]))
controller.buttonDown.pressed(lambda: arm.setSetpoint(post2Height[0], post2Height[1], post2Height[2]))
controller.buttonLeft.pressed(lambda: arm.setSetpoint(post3Height[0], post3Height[1], post3Height[2]))
controller.buttonRight.pressed(lambda: arm.setSetpoint(post1Height[0], post1Height[1], post1Height[2]))
controller.buttonX.pressed(arm.stowArm)
controller.buttonA.pressed(lambda: Ramp())
controller.buttonY.pressed(lambda: green())
def green():
    fruitColor = 0
    FruitDetection()
def Ramp():
    while inertial.orientation(ROLL) > -23:
        drivebase.drive(100, 0)
    counter = 1000
    while counter > 0:
        drivebase.drive(40, -(sideUltrasonic.distance(INCHES) - ultrasonicWallClearance) * ultrasonicWallFollowPID[0])
        if inertial.orientation(ROLL) > -5:
            counter -= 1
    pass
    drivebase.drive(0, 0)
treeNumber = 2
fruitNumber = -1
def TreeDetection():
    pass
fruitColor = 0
tracking = False
def FruitDetection():
    arm.setSetpoint(0, 30, 0)
    drivebase.drive(0, 0)
    print("Fc")
    print(fruitColor)
    timeout = 75
    continuityCounter = 0
    while timeout > 0:
        if fruitColor == 0:
            objects = camera.take_snapshot(vision_Green)
        elif fruitColor == 1:
            objects = camera.take_snapshot(vision_Orange)
        else:
            objects = camera.take_snapshot(vision_Yellow)
        print(len(objects))
        print("fc")
        print(fruitColor)
        if len(objects) == 0:
            continue
        if 'bestObject' in locals() and tracking:
            compObject = LargestVisionObject(objects)
            print("Yes")
            if (abs(compObject.centerX - bestObject.centerX) <= visionContinuityTolerance and 
                    abs(compObject.centerY - bestObject.centerY) <= visionContinuityTolerance and 
                    abs(compObject.width - bestObject.width) <= visionContinuityTolerance and
                    abs(compObject.height - bestObject.height) <= visionContinuityTolerance):
                bestObject = compObject
                continuityCounter += 1
                if continuityCounter == 5:
                    timeout = -1
                    break
            else:
                continuityCounter = 0
                tracking = False
        else:
            bestObject = LargestVisionObject(objects)
            tracking = True
    print("Stable Object")
    timeout = 2000
    while (timeout > 0):
        if fruitColor == 0:
            compObject = LargestVisionObject(camera.take_snapshot(vision_Green))
        elif fruitColor == 1:
            compObject = LargestVisionObject(camera.take_snapshot(vision_Orange))
        else:
            compObject = LargestVisionObject(camera.take_snapshot(vision_Yellow))
        drivebase.drive(0, (compObject.centerX - (cameraWidth / 2)) * -fruitAlignmentPID[0])
        timeout -= 1
    return timeout == 0            
def Harvest():
    pass
def Retreat():
    drivebase.moveLen(-6, 40)
    pass
binCoords = [0, 0, 0] #3rd is heading
def Score():
    Navigate(binCoords[0], binCoords[1], binCoords[2])
    pass
def GyroCorrection():
    pass
def GroundDetect():
    pass
def GroundHarvest():
    pass
stateFuncs = [Ramp, TreeDetection, FruitDetection, Harvest, Retreat, Score, GyroCorrection, GroundDetect, GroundHarvest]
currentState = 0
def Run():
    stateFuncs[currentState]()
def LargestVisionObject(visionObjects, byArea = False):
    best = visionObjects[0]
    visionObjects = visionObjects[1:]
    if byArea:
        for visionObject in visionObjects:
            if visionObject.height * visionObject.width > best.height * best.width:
                best = visionObject
    else:
        for visionObject in visionObjects:
            if visionObject.height > best.height:
                best = visionObject
    return best
def LargestVisionObjectIndex(visionObjects, byArea = False):
    indexBest = 0
    i = 1
    if byArea:
        while i < visionObjects.count:
            if visionObjects[i].height * visionObjects[i].width > visionObjects[indexBest].height * visionObjects[indexBest].width:
                indexBest = i
            i += 1
    else:
        while i < visionObjects.count:
            if visionObjects[i].height > visionObjects[indexBest].height:
                indexBest = i
            i += 1
    return indexBest
def Navigate(x, y, heading):
    print("Navigating")
    currentLine = ClosestLine(fieldX, fieldY)
    targetLine = ClosestLine(x, y)
    targetLinePoint = ClosestPointOnLineSegment(targetLine, x, y)
    entrance = ClosestPointOnLineSegment(currentLine, fieldX, fieldY)
    print(currentLine)
    print(targetLine)
    print(LineNav[currentLine][targetLine])
    print(entrance)
    DriveToPoint(entrance[0], entrance[1])
    i = 0
    while i < len(LineNav[currentLine][targetLine]) - 1:
        entrance = LineConnections[LineNav[currentLine][targetLine][i]][LineNav[currentLine][targetLine][i + 1]]
        print(entrance)
        DriveToPoint(entrance[0], entrance[1])
        i += 1
    print(targetLinePoint)
    DriveToPoint(targetLinePoint[0], targetLinePoint[1])
    DriveToPoint(x, y)
    drivebase.turn(heading)
def headingError(setpoint, current):
        err = setpoint - current
        while err < -180:
            err += 360
        while err > 180:
            err -= 360
        return err
def DriveToPoint(x, y):
    print("Driving")
    if not ((x - fieldX)**2 + (y - fieldY)**2 > fieldSetpointTolerance**2):
        return
    drivebase.turn(HeadingToPoint(fieldX, fieldY, x, y))
    while (x - fieldX)**2 + (y - fieldY)**2 > fieldSetpointTolerance**2:
        print(headingError(HeadingToPoint(fieldX, fieldY, x, y), 360 - inertial.orientation(YAW)))
        drivebase.drive(fieldDriveSpeed, headingError(HeadingToPoint(fieldX, fieldY, x, y), 360 - inertial.orientation(YAW)) * fieldDriveHeadingPID[0])
    print("Done Driving")
    drivebase.drive(0, 0)
def DistToLineSegment(line, xp, yp):
    if (Lines[line][2] - Lines[line][0]) * (Lines[line][2] - xp) + (Lines[line][1] - Lines[line][3]) * (Lines[line][1] - yp) < 0:
        return math.sqrt((Lines[line][2] - xp) ** 2 + (Lines[line][1] - yp) ** 2)
    elif (Lines[line][0] - Lines[line][2]) * (Lines[line][0] - xp) + (Lines[line][3] - Lines[line][1]) * (Lines[line][3] - yp) < 0:
        return math.sqrt((Lines[line][0] - xp) ** 2 + (Lines[line][3] - yp) ** 2)
    else:
        return (abs((Lines[line][3] - Lines[line][1]) * xp - (Lines[line][2] - Lines[line][0]) * 
                yp + Lines[line][2] * Lines[line][1] - Lines[line][3] * Lines[line][0]) / 
                math.sqrt((Lines[line][3] - Lines[line][1]) ** 2 + (Lines[line][2] - Lines[line][0]) ** 2))
def ClosestPointOnLineSegment(line, xp, yp):
    if (Lines[line][1] - Lines[line][3]) == 0:
        return [max(min(Lines[line][0], Lines[line][2]), min(max(Lines[line][0], Lines[line][2]), xp)), Lines[line][1]]
    elif (Lines[line][0] - Lines[line][2]) == 0:
        return [Lines[line][0], max(min(Lines[line][1], Lines[line][3]), min(max(Lines[line][1], Lines[line][3]), yp))]
    return [0, 0]
def ClosestLine(xp, yp):
    lowestIndex = 0
    lowestDist = DistToLineSegment(0, xp, yp)
    i = 1
    while i < 8:
        checkDist = DistToLineSegment(i, xp, yp)
        if checkDist < lowestDist:
            lowestIndex = i
            lowestDist = checkDist
        i += 1
    return lowestIndex
def HeadingToPoint(x1, y1, x2, y2):
    return math.atan2(y2 - y1, x2 - x1) * 360 / (2 * math.pi)
def DistanceBetweenPoints(x1, y1, x2, y2):
    return math.sqrt((y1 - y2) * (y1 - y2) + (x1 - x2) * (x1 - x2))
fieldX = 5
fieldY = 13
driveThread = Thread(drivebase.controllerDrive)
