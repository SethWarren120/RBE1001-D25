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
drivePID = [1,0,0]
turnPID = [1,0,0]
fruitHeight1 = 5 #inches
fruitHeight2 = 10 #inches
smallFruitWidth = 2.5 #inches
largeFruitWidth = 5.5 #inches
objectThreashold = 5
import math
class MecanumDrivebase ():
    def __init__(self, _motorFrontLeft, _motorFrontRight, _motorBackLeft, _motorBackRight, _gyro, _camera,
                 motorCorrectionConfig = None, _rotationUnits = DEGREES, _speedUnits = RPM):
        self.motorFrontLeft = _motorFrontLeft
        self.motorFrontRight = _motorFrontRight
        self.motorBackLeft = _motorBackLeft
        self.motorBackRight = _motorBackRight
        self.gyro = _gyro
        self.camera = _camera
        self.diameter = wheelDiameter
        self.gearing = gear_ratio
        self.wheelBase = track_width
        self.circumference = math.pi * wheelDiameter
        self.dpi = 360.0 / self.circumference
        self.rotationUnits = _rotationUnits
        self.speedUnits = _speedUnits
        self.x = 0
        self.y = 0
        self.heading = 0
        self.drivePID = drivePID
        self.turnPID = turnPID
        self.driveDuration = -1
        self.objectLocations = []
        if motorCorrectionConfig == None:
            motorCorrectionConfig = DrivebaseMotorCorrectionProfile.Disabled(_rotationUnits)
        else:
            motorCorrectionConfig.units = _rotationUnits
        odometryThread = Thread(self.updateOdometry)
        findObjectsThread = Thread(self.calculateObjectPostion)
    def drive(self, xVel, yVel, rotVel, durationSeconds=-1):
        startTime = time.time()  # Record the start time
        while durationSeconds == -1 or (time.time() - startTime < durationSeconds):
            heading = self.gyro.heading(DEGREES)
            headingRadians = math.radians(heading)  # Convert heading to radians
            tempXVel = xVel * math.cos(headingRadians) - yVel * math.sin(headingRadians)
            tempYVel = xVel * math.sin(headingRadians) + yVel * math.cos(headingRadians)
            self.motorFrontLeft.spin(FORWARD, tempYVel + tempXVel + rotVel)
            self.motorFrontRight.spin(FORWARD, tempYVel - tempXVel - rotVel)
            self.motorBackLeft.spin(FORWARD, tempYVel - tempXVel + rotVel)
            self.motorBackRight.spin(FORWARD, tempYVel + tempXVel - rotVel)
        self.motorFrontLeft.stop()
        self.motorFrontRight.stop()
        self.motorBackLeft.stop()
        self.motorBackRight.stop()
    def updateOdometry(self):
        frontLeftDistance = self.motorFrontLeft.position(DEGREES) / 360.0 * self.circumference
        frontRightDistance = self.motorFrontRight.position(DEGREES) / 360.0 * self.circumference
        backLeftDistance = self.motorBackLeft.position(DEGREES) / 360.0 * self.circumference
        backRightDistance = self.motorBackRight.position(DEGREES) / 360.0 * self.circumference
        avgX = (frontLeftDistance - frontRightDistance + backLeftDistance - backRightDistance) / 4.0
        avgY = (frontLeftDistance + frontRightDistance + backLeftDistance + backRightDistance) / 4.0
        heading = self.gyro.heading(DEGREES)
        headingRadians = math.radians(heading)
        deltaX = avgX * math.cos(headingRadians) - avgY * math.sin(headingRadians)
        deltaY = avgX * math.sin(headingRadians) + avgY * math.cos(headingRadians)
        self.x += deltaX
        self.y += deltaY
        self.heading = self.gyro.heading(DEGREES)
    def driveToPose(self, x, y, heading, tolerance=0.5):
        self.driveDuration = 0
        prevDistanceError = 0
        prevHeadingError = 0
        integralDistanceError = 0
        integralHeadingError = 0
        while True:
            deltaX = x - self.x
            deltaY = y - self.y
            distanceError = math.sqrt(deltaX**2 + deltaY**2)
            headingError = heading - self.heading
            headingError = (headingError + 180) % 360 - 180
            if distanceError <= tolerance and abs(headingError) <= tolerance:
                break
            integralDistanceError += distanceError
            derivativeDistanceError = distanceError - prevDistanceError
            positionOutput = (
                self.drivePID[0] * distanceError +
                self.drivePID[1] * integralDistanceError +
                self.drivePID[2] * derivativeDistanceError
            )
            prevDistanceError = distanceError
            integralHeadingError += headingError
            derivativeHeadingError = headingError - prevHeadingError
            headingOutput = (
                self.turnPID[0] * headingError +
                self.turnPID[1] * integralHeadingError +
                self.turnPID[2] * derivativeHeadingError
            )
            prevHeadingError = headingError
            angleToTarget = math.atan2(deltaY, deltaX)  # Angle to the target in radians
            xVel = positionOutput * math.cos(angleToTarget)  # Scale speed by angle
            yVel = positionOutput * math.sin(angleToTarget)  # Scale speed by angle
            rotVel = headingOutput  # Use heading PID output for rotation
            xVel = max(-200, min(200, xVel))
            yVel = max(-200, min(200, yVel))
            rotVel = max(-200, min(200, rotVel))
            self.drive(xVel, yVel, rotVel)
            self.updateOdometry()
        self.drive(0, 0, 0)
    def calculateObjectPostion(self):
        self.camera.takeSnapshot()
        largestObject = self.camera.largest_object
        if largestObject.exists:
            if largestObject.width > 0:
                distanceToSmallFruit = (smallFruitWidth / largestObject.width) * self.camera.focal_length
                distanceToLargeFruit = (largeFruitWidth / largestObject.width) * self.camera.focal_length
                angleToFruit = math.radians(largestObject.centerX - (self.camera.width / 2)) * self.camera.field_of_view
                smallFruitX = self.x + distanceToSmallFruit * math.cos(math.radians(self.heading) + angleToFruit)
                smallFruitY = self.y + distanceToSmallFruit * math.sin(math.radians(self.heading) + angleToFruit)
                largeFruitX = self.x + distanceToLargeFruit * math.cos(math.radians(self.heading) + angleToFruit)
                largeFruitY = self.y + distanceToLargeFruit * math.sin(math.radians(self.heading) + angleToFruit)
                self.objectLocations.append((("small", smallFruitX, smallFruitY), ("large", largeFruitX, largeFruitY)))
                self.removeRedundant()
                print(f"Fruit detected at: X={smallFruitX}, Y={smallFruitY}")
                print(f"Fruit detected at: X={largeFruitX}, Y={largeFruitY}")
        wait(20)
    def removeRedundant(self):
        uniqueLocations = []
        for fruit in self.objectLocations:
            isDuplicate = False
            for uniqueFruit in uniqueLocations:
                distance = math.sqrt((fruit[1] - uniqueFruit[1])**2 + (fruit[2] - uniqueFruit[2])**2)
                if distance < objectThreashold:
                    isDuplicate = True
                    break
            if not isDuplicate:
                uniqueLocations.append(fruit)
        self.objectLocations = uniqueLocations
class Arm ():
    def __init__(self, motor):
        self.motor = motor
class Intake ():
    def __init__(self, motor):
        self.motor = motor
class Forks ():
    def __init__(self, motor):
        self.motor = motor
brain=Brain()
fl_motor = Motor(Ports.PORT1, 18_1, True)
fr_motor = Motor(Ports.PORT9, 18_1, False)
bl_motor = Motor(Ports.PORT2, 18_1, True)
br_motor = Motor(Ports.PORT10, 18_1, False)
arm_motor = Motor(Ports.PORT11, 18_1, False)
fork_motor = Motor(Ports.PORT8, 18_1, False)
intake_motor = Motor(Ports.PORT5, 18_1, False)
inertial = Inertial(Ports.PORT3)
camera = Vision(Ports.PORT4, 50)
drivebase = MecanumDrivebase(fl_motor, fr_motor, bl_motor, br_motor, inertial, camera)
intake = Intake(intake_motor)
forks = Forks(fork_motor)
arm = Arm(arm_motor)
controller = Controller()
controller.buttonA.pressed(lambda: drivebase.driveToPose(0, 0, 0))
controller.buttonB.pressed(lambda: drivebase.driveToPose(0, 0, 90))
drivebase.drive(controller.axis3, controller.axis4, controller.axis2)
