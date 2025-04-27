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
vision_orange = Colordesc(1,245,134,89,5,0.17)
cameraOffset = [0, 0, 0, 0, 0, 0] #inches
cameraFOV = 75 #degrees
cameraWidth = 320
cameraHeight = 240
cameraXOffset = -cameraWidth/2
cameraYOffset = -cameraHeight/2
fruitHeight1 = 5 #inches
fruitHeight2 = 10 #inches
smallFruitWidth = 2.5 #inches
largeFruitWidth = 5.5 #inches
objectThreashold = 5
tag1 = [0,0,0]
tag2 = [0,0,0]
tag3 = [0,0,0]
tag4 = [0,0,0]
tagLocations = [tag1,
                tag2,
                tag3,
                tag4]
post1Location = [0,0]
post2Location = [0,0]
post3Location = [0,0]
posts = [post1Location,
          post2Location,
          post3Location]
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
maxArmLength = 108 
minArmAngle = 0
maxArmAngle = 117
minWristAngle = 0
maxWristAngle = 180
armGearRatio = 5
pivotGearRatio = 72/12
wristGearRatio = 5
armPID = [1,0,0]
armTolerance = 1
pivotPID = [1.4,0,0]
pivotFF = 0.5
pivotTolerance = 0.5
wristPID = [1,0,0]
wristTolerance = 1
import math
class MecanumDrivebase ():
    def __init__(self, _motorFrontLeft: Motor, _motorFrontRight: Motor, _motorBackLeft: Motor, _motorBackRight: Motor, 
                 _gyro: Inertial, _camera: AiVision, _rotationUnits = DEGREES, _speedUnits = RPM):
        self.motorFrontLeft = _motorFrontLeft
        self.motorFrontRight = _motorFrontRight
        self.motorBackLeft = _motorBackLeft
        self.motorBackRight = _motorBackRight
        self.gyro = _gyro
        self.camera = _camera
        self.rotationUnits = _rotationUnits
        self.speedUnits = _speedUnits
        self.x = 0
        self.y = 0
        self.heading = 0
        self.objectLocations = []
        self.driveOverride = False
        odometryThread = Thread(self.updateOdometry)
    def drive(self, xVel, yVel, rotVel):
        if not self.driveOverride:
            self.driveFree = False
            rotationFactor = (wheel_base + track_width) / 2.0
            heading = self.gyro.heading(DEGREES)  # Returns [0, 360)
            headingRadians = math.radians(heading)  # Convert heading to radians
            tempXVel = xVel * math.cos(headingRadians) - yVel * math.sin(headingRadians)
            tempYVel = xVel * math.sin(headingRadians) + yVel * math.cos(headingRadians)
            distanceError = math.sqrt(tempXVel**2 + tempYVel**2)
            integralDistanceError = 0
            derivativeDistanceError = distanceError - self.prevDistanceError if hasattr(self, 'prevDistanceError') else 0
            integralDistanceError += distanceError
            linearOutput = (
                drivePID[0] * distanceError +
                drivePID[1] * integralDistanceError +
                drivePID[2] * derivativeDistanceError
            )
            self.prevDistanceError = distanceError  # Store for next call
            scaledXVel = linearOutput * (tempXVel / distanceError) if distanceError != 0 else 0
            scaledYVel = linearOutput * (tempYVel / distanceError) if distanceError != 0 else 0
            rotVel = rotVel % 360
            headingError = rotVel - heading
            headingError = (headingError + 180) % 360 - 180  # Normalize to [-180, 180]
            integralHeadingError = 0
            derivativeHeadingError = headingError - self.prevHeadingError if hasattr(self, 'prevHeadingError') else 0
            integralHeadingError += headingError
            rotationalOutput = (
                turnPID[0] * headingError +
                turnPID[1] * integralHeadingError +
                turnPID[2] * derivativeHeadingError
            )
            self.prevHeadingError = headingError  # Store for next call
            self.motorFrontLeft.spin(FORWARD, scaledYVel + scaledXVel + rotationalOutput * rotationFactor)
            self.motorFrontRight.spin(FORWARD, scaledYVel - scaledXVel - rotationalOutput * rotationFactor)
            self.motorBackLeft.spin(FORWARD, scaledYVel - scaledXVel + rotationalOutput * rotationFactor)
            self.motorBackRight.spin(FORWARD, scaledYVel + scaledXVel - rotationalOutput * rotationFactor)
            self.driveFree = True
    def updateOdometry(self):
        frontLeftDistance = self.motorFrontLeft.position(DEGREES) / 360.0 * wheelCircumference
        frontRightDistance = self.motorFrontRight.position(DEGREES) / 360.0 * wheelCircumference
        backLeftDistance = self.motorBackLeft.position(DEGREES) / 360.0 * wheelCircumference
        backRightDistance = self.motorBackRight.position(DEGREES) / 360.0 * wheelCircumference
        avgX = (frontLeftDistance - frontRightDistance + backLeftDistance - backRightDistance) / 4.0
        avgY = (frontLeftDistance + frontRightDistance + backLeftDistance + backRightDistance) / 4.0
        heading = self.gyro.heading(DEGREES)
        headingRadians = math.radians(heading)
        deltaX = avgX * math.cos(headingRadians) - avgY * math.sin(headingRadians)
        deltaY = avgX * math.sin(headingRadians) + avgY * math.cos(headingRadians)
        self.x += deltaX
        self.y += deltaY
        self.heading = self.gyro.heading(DEGREES)
        aprilTags = self.camera.take_snapshot(AiVision.ALL_TAGS)
        for tag in aprilTags:
            location = tagLocations[tag.id-1]
            tagX, tagY, tagAngle = location
            observedX = tag.centerX
            observedY = tag.centerY
            observedAngle = tag.angle
            adjustedX = tagX - (cameraOffset[0] * math.cos(math.radians(self.heading)) - cameraOffset[1] * math.sin(math.radians(self.heading)))
            adjustedY = tagY - (cameraOffset[0] * math.sin(math.radians(self.heading)) + cameraOffset[1] * math.cos(math.radians(self.heading)))
            self.x = adjustedX - observedX
            self.y = adjustedY - observedY
            self.heading = (tagAngle - observedAngle) % 360
    def driveToPose(self, x, y, heading, tolerance=0.5):
        self.driveOverride = True
        while not self.driveFree:
            pass
        heading = heading % 360
        prevDistanceError = 0
        prevHeadingError = 0
        integralDistanceError = 0
        integralHeadingError = 0
        waypoints = self.checkPathAndAvoidObstacles(self.x, self.y, x, y)
        currentWaypoint = 0
        while currentWaypoint < len(waypoints):
            targetX, targetY = waypoints[currentWaypoint]
            while True:
                deltaX = targetX - self.x
                deltaY = targetY - self.y
                distanceError = math.sqrt(deltaX**2 + deltaY**2)
                if currentWaypoint == len(waypoints) - 1:
                    targetHeading = heading
                else:
                    targetHeading = math.degrees(math.atan2(deltaY, deltaX))
                headingError = targetHeading - self.heading
                headingError = (headingError + 180) % 360 - 180
                waypointTolerance = tolerance if currentWaypoint == len(waypoints) - 1 else 2.0
                headingTolerance = tolerance if currentWaypoint == len(waypoints) - 1 else 10.0
                if distanceError <= waypointTolerance and abs(headingError) <= headingTolerance:
                    break
                integralDistanceError += distanceError
                derivativeDistanceError = distanceError - prevDistanceError
                positionOutput = (
                    drivePID[0] * distanceError +
                    drivePID[1] * integralDistanceError +
                    drivePID[2] * derivativeDistanceError
                )
                prevDistanceError = distanceError
                integralHeadingError += headingError
                derivativeHeadingError = headingError - prevHeadingError
                headingOutput = (
                    turnPID[0] * headingError +
                    turnPID[1] * integralHeadingError +
                    turnPID[2] * derivativeHeadingError
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
                wait(20)
            currentWaypoint += 1
        self.driveFree = True
        self.driveOverride = False
    def checkPathAndAvoidObstacles(self, startX, startY, endX, endY):
        safetyDistance = 3.0  # inches
        needsPathPlanning = False
        closestPost = None
        minDistance = float('inf')
        for post in posts:    
            lineLength = math.sqrt((endX - startX)**2 + (endY - startY)**2)
            if lineLength == 0:  # Start and end points are the same
                distance = math.sqrt((post[0] - startX)**2 + (post[1] - startY)**2)
            else:
                distance = abs((endY - startY) * post[0] - (endX - startX) * post[1] + 
                            endX * startY - endY * startX) / lineLength
                t = ((post[0] - startX) * (endX - startX) + 
                    (post[1] - startY) * (endY - startY)) / (lineLength**2)
                if t < 0:
                    distance = math.sqrt((post[0] - startX)**2 + (post[1] - startY)**2)
                elif t > 1:
                    distance = math.sqrt((post[0] - endX)**2 + (post[1] - endY)**2)
            if distance < minDistance:
                minDistance = distance
                closestPost = post
        if minDistance < safetyDistance:
            needsPathPlanning = True
        waypoints = []
        if needsPathPlanning and closestPost is not None:
            lineVector = [endX - startX, endY - startY]
            lineLength = math.sqrt(lineVector[0]**2 + lineVector[1]**2)
            unitVector = [lineVector[0] / lineLength, lineVector[1] / lineLength]
            postToStart = [closestPost[0] - startX, closestPost[1] - startY]
            projectionLength = (postToStart[0] * unitVector[0] + postToStart[1] * unitVector[1])
            nearestPoint = [startX + projectionLength * unitVector[0], 
                            startY + projectionLength * unitVector[1]]
            perpVector = [-unitVector[1], unitVector[0]]
            sideDeterminer = 0
            for otherPost in posts:
                if otherPost != closestPost and not (otherPost[0] == 0 and otherPost[1] == 0):
                    vectorToOtherPost = [otherPost[0] - nearestPoint[0], otherPost[1] - nearestPoint[1]]
                    dotProduct = vectorToOtherPost[0] * perpVector[0] + vectorToOtherPost[1] * perpVector[1]
                    sideDeterminer += dotProduct
            if sideDeterminer < 0:
                perpVector = [-perpVector[0], -perpVector[1]]
            avoidanceDistance = safetyDistance * 1.5
            avoidancePoint = [nearestPoint[0] + perpVector[0] * avoidanceDistance, 
                            nearestPoint[1] + perpVector[1] * avoidanceDistance]
            waypoints.append((avoidancePoint[0], avoidancePoint[1]))
        waypoints.append((endX, endY))
        return waypoints
    def zeroGyro(self):
        self.gyro.set_heading(0, DEGREES)
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
        armLengthThread = Thread(lambda: self.setLength())
        armAngleThread = Thread(lambda: self.setAngle())
        wristAngleThread = Thread(lambda: self.setWristAngle())
    def getLength(self):
        self.armLength = self.armmotorL.position(DEGREES) / armGearRatio
        return self.armLength
    def setLength(self):
        targetLength = self.clamp(self.dLength, minArmLength, maxArmLength)
        kp, ki, kd = armPID
        integral = 0
        prevError = 0
        while True:
            currentLength = self.getLength()
            error = targetLength - currentLength
            integral += error
            derivative = error - prevError
            output = kp * error + ki * integral + kd * derivative
            self.armmotorR.spin(FORWARD, output, PERCENT)
            self.armmotorL.spin(FORWARD, output, PERCENT)
            prevError = error
            self.armLength = self.armmotorL.position(DEGREES) / armGearRatio
        self.armmotorL.stop()
        self.armmotorR.stop()
    def setAngle(self):
        targetAngle = self.clamp(self.dAngle, minArmAngle, maxArmAngle)
        kp, ki, kd = pivotPID
        integral = 0
        prevError = 0
        while True:
            currentAngle = self.getAngle()
            error = targetAngle - currentAngle
            integral += error
            derivative = error - prevError
            output = (kp * error + pivotFF*math.cos(math.radians(self.getAngle()))) + ki * integral + kd * derivative
            self.pivotmotorL.spin(FORWARD, output, PERCENT)
            self.pivotmotorR.spin(FORWARD, output, PERCENT)
            prevError = error
            self.armAngle = self.pivotmotorL.position(DEGREES) / pivotGearRatio
        self.pivotmotorL.stop()
        self.pivotmotorR.stop()
    def getAngle(self):
        self.armAngle = self.pivotmotorL.position(DEGREES) / pivotGearRatio
        return self.armAngle
    def getWristAngle(self):
        self.wristAngle = self.wristmotor.position(DEGREES) / wristGearRatio
        return self.wristmotor.position(DEGREES) / wristGearRatio
    def setWristAngle(self):
        targetAngle = self.clamp(self.dWrist, minWristAngle, maxWristAngle)
        kp, ki, kd = wristPID
        integral = 0
        prevError = 0
        while True:
            currentAngle = self.getWristAngle()
            error = targetAngle - currentAngle
            integral += error
            derivative = error - prevError
            output = kp * error + ki * integral + kd * derivative
            self.wristmotor.spin(FORWARD, output, PERCENT)
            prevError = error
            self.wristAngle = self.wristmotor.position(DEGREES) / wristGearRatio
        self.wristmotor.stop()
    def zeroLength(self):
        while True:
            self.armmotorL.spin(REVERSE, 10, PERCENT)
            self.armmotorR.spin(REVERSE, 10, PERCENT)
            if self.armmotorL.current() >= 1 or self.armmotorL.current() >= 1:
                self.armmotorL.stop()
                self.armmotorR.stop()
                self.armLength = 0
                self.armmotorR.set_position(0, DEGREES)
                self.armmotorL.set_position(0, DEGREES)
                break
    def zeroAngle(self):
        while True:
            self.pivotmotorL.spin(REVERSE, 10, PERCENT)
            self.pivotmotorR.spin(REVERSE, 10, PERCENT)
            if self.pivotmotorR.current() >= 1 or self.pivotmotorL.current() >= 1:
                self.pivotmotorL.stop()
                self.pivotmotorR.stop()
                self.armAngle = 0
                self.pivotmotorR.set_position(0, DEGREES)
                self.pivotmotorL.set_position(0, DEGREES)
                break
    def zeroWrist(self):
        while True:
            self.wristmotor.spin(REVERSE, 10, PERCENT)
            if self.wristmotor.current() >= 1:
                self.wristmotor.stop()
                self.wristAngle = 0
                self.wristmotor.set_position(0, DEGREES)
                break
    def zeroing(self):
        self.zeroAngle()
    def clamp(self, value, min_value, max_value):
        return max(min_value, min(value, max_value))
    def setSetpoint(self, length, angle, wristAngle):
        self.dLength = length
        self.dAngle = angle
        self.dWrist = wristAngle
class Intake ():
    def __init__(self, intakemotor: Motor):
        self.intakemotor = intakemotor
        self.escapeLoop = False
    def runIntake(self, direction):
        if direction == "forward":
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
class Forks():
    def __init__(self, forkmotor: Motor):
        self.forkmotor = forkmotor
        self.forksDeployed = False
        self.basketContains = []
        self.nextSlot = 0,0
        self.currentSlot = 0
    def deployForks(self):
        self.forkmotor.spin_for(FORWARD, 90 * forksGearRatio, DEGREES)
        self.forksDeployed = True
    def retractForks(self):
        self.forkmotor.spin_for(FORWARD, -90 * forksGearRatio, DEGREES)
        self.forksDeployed = False
    def toggleForks(self):
        if self.forksDeployed:
            self.retractForks()
        else:
            self.deployForks()
    def addObject(self, row, column):
        if len(self.basketContains) < row-1:
            if (column == 1):
                self.basketContains.append((1, 0))
            else:
                self.basketContains.append((0, 1))
        else:
            if (column == 1):
                self.basketContains[row-1] = (1, 0)
            else:
                self.basketContains[row-1] = (0, 1)
        self.nextSlot = self.getLeftestSlot()
    def getLeftestSlot(self):
        for i in range(len(self.basketContains)):
            for j in range(len(self.basketContains)):
                if self.basketContains[i][j] == 0:
                    return i+1,j+1
        return -1,-1
def setSubsystems(driveBase: MecanumDrivebase, arm: Arm, intake: Intake, forks: Forks):
    global driveSub, armSub, intakeSub, forksSub
    driveSub = driveBase
    armSub = arm
    intakeSub = intake
    forksSub = forks
def grabFruit(fruitPose, length, isLowFruit):
    if isLowFruit:
        armSub.setSetpoint(length, lowFruitAngle, lowFruitWristAngle)
    else:
        armSub.setSetpoint(length, highFruitAngle, highFruitWristAngle)
    intakeSub.intakeUntilCurrent()
    armLength = armSub.getLength()  # Get the current arm length
    armAngle = math.radians(armSub.getAngle())  # Convert arm angle to radians
    robotHeading = math.radians(driveSub.heading) # uses the heading in radians
    offsetX = armLength * math.cos(robotHeading) # Get the x offset of the arm relative to the robot
    offsetY = armLength * math.sin(robotHeading) # Get the y offset of the arm relative to the robot
    adjustedX = fruitPose[0] - offsetX
    adjustedY = fruitPose[1] - offsetY
    deltaX = fruitPose[0] - driveSub.x  # get the change in x position
    deltaY = fruitPose[1] - driveSub.y  # get the change in y position
    targetHeading = math.degrees(math.atan2(deltaY, deltaX))  # find the angle the robot neeeds to turn to face the fruit
    driveSub.driveToPose(adjustedX, adjustedY, targetHeading)
def stowArm():
    intakeSub.stopIntake()
    armSub.setSetpoint(minArmLength, minArmAngle, minWristAngle)
brain=Brain()
fl_motor = Motor(Ports.PORT1, 18_1, True)
fr_motor = Motor(Ports.PORT9, 18_1, False)
bl_motor = Motor(Ports.PORT2, 18_1, True)
br_motor = Motor(Ports.PORT10, 18_1, False)
arm_motorL = Motor(Ports.PORT14, 18_1, False)
arm_motorR = Motor(Ports.PORT15, 18_1, True)
wrist_motor = Motor(Ports.PORT11, 18_1, False)
pivot_motorL = Motor(Ports.PORT17, 18_1, True)
pivot_motorR = Motor(Ports.PORT16, 18_1, False)
intake_motor = Motor(Ports.PORT5, 18_1, False)
fork_motor = Motor(Ports.PORT8, 18_1, False)
inertial = Inertial(Ports.PORT3)
camera = AiVision(Ports.PORT4, 50)
camera.start_awb()
arm = Arm(arm_motorL, arm_motorR, pivot_motorL, pivot_motorR, wrist_motor)
controller = Controller()
def printDebugging():
    while True:
        brain.screen.print_at(arm.getLength(), x=1, y=60)
        brain.screen.print_at(arm.getAngle(), x=1, y=80)
        brain.screen.print_at(arm.getWristAngle(), x=1, y=100)
        sleep(100)
debugThread = Thread(printDebugging)
arm.setSetpoint(100, 90, 0)
