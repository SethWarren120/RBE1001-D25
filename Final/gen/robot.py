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
drivePID = [0.1,0,0]
turnPID = [0.1,0,0]
cameraOffset = [0, 0, 0, 0, 0, 0] #inches
vision_orange = Colordesc(1,245,134,89,5,0.17)
vision_yellow = Colordesc(1,245,134,89,5,0.17)
vision_green = Colordesc(1,245,134,89,5,0.17)
cameraWidth = 320
cameraHeight = 240
cameraXOffset = cameraWidth/2
cameraYOffset = cameraHeight/2
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
minArmLength = 2
maxArmLength = 108 
minArmAngle = 1
maxArmAngle = 70
minWristAngle = 2
maxWristAngle = 180
armGearRatio = 5
pivotGearRatio = 72/12 * 20/12
wristGearRatio = 5
armPID = [1,0,0]
armTolerance = 1
pivotPID = [7.5,0,0]
pivotFF = 1
pivotTolerance = 0.5
wristPID = [1,0,0]
wristTolerance = 1
import math
class DriveState():
    IDLE = 0
    TELEOP = 1
    AUTO = 2
class MecanumDrivebase ():
    def __init__(self, _motorFrontLeft: Motor, _motorFrontRight: Motor, _motorBackLeft: Motor, _motorBackRight: Motor, 
                 _gyro: Inertial, _tagCamera: AiVision, _camera: Vision,_controller: Controller, _rotationUnits = DEGREES, _speedUnits = RPM):
        self.motorFrontLeft = _motorFrontLeft
        self.motorFrontRight = _motorFrontRight
        self.motorBackLeft = _motorBackLeft
        self.motorBackRight = _motorBackRight
        self.gyro = _gyro
        self.tagCamera = _tagCamera
        self.camera = _camera
        self.controller = _controller
        self.rotationUnits = _rotationUnits
        self.speedUnits = _speedUnits
        self.x = 0
        self.y = 0
        self.heading = 0
        self.targetX = 0
        self.targetY = 0
        self.targetHeading = 0
        self.tolerance = 0.5
        self.state = DriveState.IDLE
        odometryThread = Thread(self.updateOdometry)
        stateUpdateThread = Thread(self.stateUpdate)
        controllerThread = Thread(self.controllerDriveUpdate)
    def setState(self, newState):
        if self.state != newState:
            self.state = newState
            if newState == DriveState.IDLE:
                self.stopMotors()
            elif newState == DriveState.AUTO:
                self.prevDistanceError = 0
                self.prevHeadingError = 0
                self.integralDistanceError = 0
                self.integralHeadingError = 0
    def stateUpdate(self):
        while True:
            if self.state == DriveState.IDLE:
                pass
            elif self.state == DriveState.AUTO:
                self.updateAutoDrive()
            wait(20)
    def stopMotors(self):
        self.motorFrontLeft.stop()
        self.motorFrontRight.stop()
        self.motorBackLeft.stop()
        self.motorBackRight.stop()
    def drive(self, xVel, yVel, rotVel):
        if self.state == DriveState.AUTO:
            return
        if xVel != 0 or yVel != 0 or rotVel != 0:
            self.setState(DriveState.TELEOP)
        else:
            self.setState(DriveState.IDLE)
            return
        linearFactor = 5.0
        rotationFactor = (wheel_base + track_width) / 2.0
        heading = self.gyro.heading(DEGREES)  # Returns [0, 360)
        headingRadians = math.radians(heading)  # Convert heading to radians
        newXVel = (xVel * math.cos(headingRadians) - yVel * math.sin(headingRadians))*linearFactor
        newYVel = (xVel * math.sin(headingRadians) + yVel * math.cos(headingRadians))**linearFactor
        rotVel = rotVel % 360
        print(newYVel + newXVel + rotVel * rotationFactor)
        print(newYVel - newXVel - rotVel * rotationFactor)
        print(newYVel - newXVel + rotVel * rotationFactor)
        print(newYVel + newXVel - rotVel * rotationFactor)
        self.motorFrontLeft.spin(FORWARD, newYVel + newXVel + rotVel * rotationFactor)
        self.motorFrontRight.spin(FORWARD, newYVel - newXVel - rotVel * rotationFactor)
        self.motorBackLeft.spin(FORWARD, newYVel - newXVel + rotVel * rotationFactor)
        self.motorBackRight.spin(FORWARD, newYVel + newXVel - rotVel * rotationFactor)
    def controllerDriveUpdate(self):
        while True:
            if self.state != DriveState.AUTO:
                xInput = self.controller.axis4.position()  # Left joystick X
                yInput = self.controller.axis3.position()  # Left joystick Y
                rotInput = self.controller.axis1.position()  # Right joystick X
                deadzone = 5
                if abs(xInput) < deadzone: xInput = 0
                if abs(yInput) < deadzone: yInput = 0
                if abs(rotInput) < deadzone: rotInput = 0
                self.drive(xInput, yInput, rotInput)
            wait(20)
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
        aprilTags = self.tagCamera.take_snapshot(AiVision.ALL_TAGS)
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
        self.targetX = x
        self.targetY = y
        self.targetHeading = heading % 360
        self.tolerance = tolerance
        self.setState(DriveState.AUTO)
        self.prevDistanceError = 0
        self.prevHeadingError = 0
        self.integralDistanceError = 0
        self.integralHeadingError = 0
    def checkPathAndAvoidObstacles(self, startX, startY, endX, endY):
        safetyDistance = 1.0
        needsPathPlanning = False
        closestPost = None
        minDistance = float('inf')
        for post in posts:    
            lineLength = math.sqrt((endX - startX)**2 + (endY - startY)**2)
            if lineLength == 0:
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
    def updateAutoDrive(self):
        waypoints = self.checkPathAndAvoidObstacles(self.x, self.y, self.targetX, self.targetY)
        currentWaypoint = 0
        while currentWaypoint < len(waypoints) and self.state == DriveState.AUTO:
            targetX, targetY = waypoints[currentWaypoint]
            deltaX = targetX - self.x
            deltaY = targetY - self.y
            distanceError = math.sqrt(deltaX**2 + deltaY**2)
            if currentWaypoint == len(waypoints) - 1:
                targetHeading = self.targetHeading
            else:
                targetHeading = math.degrees(math.atan2(deltaY, deltaX))
            headingError = targetHeading - self.heading
            headingError = (headingError + 180) % 360 - 180
            waypointTolerance = self.tolerance if currentWaypoint == len(waypoints) - 1 else 2.0
            headingTolerance = self.tolerance if currentWaypoint == len(waypoints) - 1 else 10.0
            if distanceError <= waypointTolerance and abs(headingError) <= headingTolerance:
                currentWaypoint += 1
                continue
            self.integralDistanceError += distanceError
            derivativeDistanceError = distanceError - self.prevDistanceError
            positionOutput = (
                drivePID[0] * distanceError +
                drivePID[1] * self.integralDistanceError +
                drivePID[2] * derivativeDistanceError
            )
            self.prevDistanceError = distanceError
            self.integralHeadingError += headingError
            derivativeHeadingError = headingError - self.prevHeadingError
            headingOutput = (
                turnPID[0] * headingError +
                turnPID[1] * self.integralHeadingError +
                turnPID[2] * derivativeHeadingError
            )
            self.prevHeadingError = headingError
            angleToTarget = math.atan2(deltaY, deltaX)  # Angle to the target in radians
            xVel = positionOutput * math.cos(angleToTarget)  # Scale speed by angle
            yVel = positionOutput * math.sin(angleToTarget)  # Scale speed by angle
            rotVel = headingOutput  # Use heading PID output for rotation
            xVel = max(-200, min(200, xVel))
            yVel = max(-200, min(200, yVel))
            rotVel = max(-200, min(200, rotVel))
            heading = self.gyro.heading(DEGREES)
            headingRadians = math.radians(heading)
            tempXVel = xVel * math.cos(headingRadians) - yVel * math.sin(headingRadians)
            tempYVel = xVel * math.sin(headingRadians) + yVel * math.cos(headingRadians)
            rotationFactor = (wheel_base + track_width) / 2.0
            self.motorFrontLeft.spin(FORWARD, tempYVel + tempXVel + rotVel * rotationFactor)
            self.motorFrontRight.spin(FORWARD, tempYVel - tempXVel - rotVel * rotationFactor)
            self.motorBackLeft.spin(FORWARD, tempYVel - tempXVel + rotVel * rotationFactor)
            self.motorBackRight.spin(FORWARD, tempYVel + tempXVel - rotVel * rotationFactor)
            wait(20)  # Small delay for control loop
        if self.state == DriveState.AUTO and currentWaypoint >= len(waypoints):
            self.setState(DriveState.IDLE)
    def zeroGyro(self):
        self.gyro.set_heading(0, DEGREES)
    def centerToObject(self):
        orangeObjects = self.camera.take_snapshot(vision_orange)
        orangeClosest = self.camera.largest_object()
        greenObjects = self.camera.take_snapshot(vision_green)
        greenClosest = self.camera.largest_object()
        yellowObjects = self.camera.take_snapshot(vision_yellow)
        yellowClosest = self.camera.largest_object()
        objectToCenter = None
        if (orangeClosest.width > greenClosest.width and orangeClosest.width > yellowClosest.width):
            objectToCenter = orangeClosest
        elif (greenClosest.width > orangeClosest.width and greenClosest.width > yellowClosest.width):
            objectToCenter = greenClosest
        elif (yellowClosest.width > orangeClosest.width and yellowClosest.width > greenClosest.width):
            objectToCenter = yellowClosest
        else:
            pass
        if (objectToCenter is not None):
            objectCenterX = objectToCenter.centerX - cameraXOffset
            objectCenterY = objectToCenter.centerY - cameraYOffset
            angleToTurn = math.atan2(objectCenterY, objectCenterX) * (180 / math.pi)
            self.driveToPose(self.x, self.y, angleToTurn)
class Arm ():
    def __init__(self, armmotorL: Motor, armmotorR: Motor, pivotmotor: Motor, wristmotor: Motor):
        self.armmotorL = armmotorL
        self.armmotorR = armmotorR
        self.pivotmotor = pivotmotor
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
            self.armmotorR.spin(FORWARD, output, PERCENT)
            self.armmotorL.spin(FORWARD, output, PERCENT)
            prevError = error
            self.armLength = self.armmotorL.position(DEGREES) / armGearRatio
    def getAngle(self):
        self.armAngle = self.pivotmotor.position(DEGREES) / pivotGearRatio
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
            self.pivotmotor.spin(FORWARD, output, PERCENT)
            prevError = error
            self.armAngle = self.pivotmotor.position(DEGREES) / pivotGearRatio
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
def setSubsystems(driveBase: MecanumDrivebase, arm: Arm, intake: Intake):
    global driveSub, armSub, intakeSub, forksSub
    driveSub = driveBase
    armSub = arm
    intakeSub = intake
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
fl_motor = Motor(Ports.PORT10, 18_1, False)
fr_motor = Motor(Ports.PORT20, 18_1, True)
bl_motor = Motor(Ports.PORT1, 18_1, False)
br_motor = Motor(Ports.PORT11, 18_1, True)
arm_motorL = Motor(Ports.PORT14, 18_1, False)
arm_motorR = Motor(Ports.PORT15, 18_1, True)
wrist_motor = Motor(Ports.PORT13, 18_1, False)
pivot_motor = Motor(Ports.PORT12, 18_1, False)
intake_motor = Motor(Ports.PORT5, 18_1, False)
inertial = Inertial(Ports.PORT3)
camera = Vision(Ports.PORT6, 50)
tagCamera = AiVision(Ports.PORT4, 50)
tagCamera.start_awb()
controller = Controller()
drivebase = MecanumDrivebase(fl_motor, fr_motor, bl_motor, br_motor, inertial, tagCamera, camera, controller)
intake = Intake(intake_motor)
arm = Arm(arm_motorL, arm_motorR, pivot_motor, wrist_motor)
setSubsystems(drivebase, arm, intake)
def printDebugging():
    while True:
        brain.screen.print_at(arm.getLength(), x=1, y=60)
        brain.screen.print_at(arm.getAngle(), x=1, y=80)
        brain.screen.print_at(arm.getWristAngle(), x=1, y=100)
        sleep(20)
debugThread = Thread(printDebugging)
arm.setSetpoint(0, 10, 0)
