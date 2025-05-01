# Library imports
from vex import *
from Subsystems.Drivebase.drivebaseMotorCorrector import *
import math
from constants import *

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
            
            # elif self.state == DriveState.TELEOP:
                # self.controllerDriveUpdate()

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

        # Get the current heading from the gyro
        heading = self.gyro.heading(DEGREES)  # Returns [0, 360)
        headingRadians = math.radians(heading)  # Convert heading to radians

        # Apply field-centric transformation
        newXVel = (xVel * math.cos(headingRadians) - yVel * math.sin(headingRadians))*linearFactor
        newYVel = (xVel * math.sin(headingRadians) + yVel * math.cos(headingRadians))**linearFactor

        # Ensure rotVel is in [0, 360) for consistent comparison
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
                # Get joystick positions - adjust these based on your controller layout
                xInput = self.controller.axis4.position()  # Left joystick X
                yInput = self.controller.axis3.position()  # Left joystick Y
                rotInput = self.controller.axis1.position()  # Right joystick X
                
                # Apply deadzone to prevent drift
                deadzone = 5
                if abs(xInput) < deadzone: xInput = 0
                if abs(yInput) < deadzone: yInput = 0
                if abs(rotInput) < deadzone: rotInput = 0
                
                # Pass to drive method
                self.drive(xInput, yInput, rotInput)
            
            wait(20)

    def updateOdometry(self):
        frontLeftDistance = self.motorFrontLeft.position(DEGREES) / 360.0 * wheelCircumference
        frontRightDistance = self.motorFrontRight.position(DEGREES) / 360.0 * wheelCircumference
        backLeftDistance = self.motorBackLeft.position(DEGREES) / 360.0 * wheelCircumference
        backRightDistance = self.motorBackRight.position(DEGREES) / 360.0 * wheelCircumference

        # Average the distances to calculate the robot's movement in local coordinates
        avgX = (frontLeftDistance - frontRightDistance + backLeftDistance - backRightDistance) / 4.0
        avgY = (frontLeftDistance + frontRightDistance + backLeftDistance + backRightDistance) / 4.0

        # Get the current heading from the gyro
        heading = self.gyro.heading(DEGREES)
        headingRadians = math.radians(heading)

        # Transform local movement to field coordinates
        deltaX = avgX * math.cos(headingRadians) - avgY * math.sin(headingRadians)
        deltaY = avgX * math.sin(headingRadians) + avgY * math.cos(headingRadians)

        # Update the robot's position
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
        
        # Reset PID errors
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
                
                # If projection is outside line segment, use distance to closest endpoint
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
                    # Vector from nearest point to other post
                    vectorToOtherPost = [otherPost[0] - nearestPoint[0], otherPost[1] - nearestPoint[1]]
                    # Dot product with perpendicular vector to determine side
                    dotProduct = vectorToOtherPost[0] * perpVector[0] + vectorToOtherPost[1] * perpVector[1]
                    sideDeterminer += dotProduct
            
            # If sideDeterminer is negative, flip the perpendicular vector
            if sideDeterminer < 0:
                perpVector = [-perpVector[0], -perpVector[1]]
            
            # Calculate avoidance waypoint (safety distance + a bit more for margin)
            avoidanceDistance = safetyDistance * 1.5
            avoidancePoint = [nearestPoint[0] + perpVector[0] * avoidanceDistance, 
                            nearestPoint[1] + perpVector[1] * avoidanceDistance]
            
            # Generate waypoints: first to avoidance point, then to destination
            waypoints.append((avoidancePoint[0], avoidancePoint[1]))
        
        # If no waypoints needed, go straight to destination
        waypoints.append((endX, endY))
        return waypoints

    def updateAutoDrive(self):
        # Calculate waypoints to avoid obstacles
        waypoints = self.checkPathAndAvoidObstacles(self.x, self.y, self.targetX, self.targetY)
        currentWaypoint = 0
        
        while currentWaypoint < len(waypoints) and self.state == DriveState.AUTO:
            targetX, targetY = waypoints[currentWaypoint]
            
            # Calculate the error in position and heading
            deltaX = targetX - self.x
            deltaY = targetY - self.y
            distanceError = math.sqrt(deltaX**2 + deltaY**2)
            
            # Use final heading for the last waypoint, otherwise face toward waypoint
            if currentWaypoint == len(waypoints) - 1:
                targetHeading = self.targetHeading
            else:
                # Point toward the waypoint
                targetHeading = math.degrees(math.atan2(deltaY, deltaX))
            
            headingError = targetHeading - self.heading
            headingError = (headingError + 180) % 360 - 180

            # Check if the robot is within the tolerances of the current waypoint
            waypointTolerance = self.tolerance if currentWaypoint == len(waypoints) - 1 else 2.0
            headingTolerance = self.tolerance if currentWaypoint == len(waypoints) - 1 else 10.0
            
            if distanceError <= waypointTolerance and abs(headingError) <= headingTolerance:
                currentWaypoint += 1
                continue

            # PID for position
            self.integralDistanceError += distanceError
            derivativeDistanceError = distanceError - self.prevDistanceError
            positionOutput = (
                drivePID[0] * distanceError +
                drivePID[1] * self.integralDistanceError +
                drivePID[2] * derivativeDistanceError
            )
            self.prevDistanceError = distanceError

            # PID for heading
            self.integralHeadingError += headingError
            derivativeHeadingError = headingError - self.prevHeadingError
            headingOutput = (
                turnPID[0] * headingError +
                turnPID[1] * self.integralHeadingError +
                turnPID[2] * derivativeHeadingError
            )
            self.prevHeadingError = headingError

            # Calculate the desired velocities
            angleToTarget = math.atan2(deltaY, deltaX)  # Angle to the target in radians
            xVel = positionOutput * math.cos(angleToTarget)  # Scale speed by angle
            yVel = positionOutput * math.sin(angleToTarget)  # Scale speed by angle
            rotVel = headingOutput  # Use heading PID output for rotation

            # Limit the velocities to the maximum speed
            xVel = max(-200, min(200, xVel))
            yVel = max(-200, min(200, yVel))
            rotVel = max(-200, min(200, rotVel))
            
            # Apply velocities directly to motors
            heading = self.gyro.heading(DEGREES)
            headingRadians = math.radians(heading)
            
            # Apply field-centric transformation
            tempXVel = xVel * math.cos(headingRadians) - yVel * math.sin(headingRadians)
            tempYVel = xVel * math.sin(headingRadians) + yVel * math.cos(headingRadians)
            
            rotationFactor = (wheel_base + track_width) / 2.0
            
            # Drive the motors
            self.motorFrontLeft.spin(FORWARD, tempYVel + tempXVel + rotVel * rotationFactor)
            self.motorFrontRight.spin(FORWARD, tempYVel - tempXVel - rotVel * rotationFactor)
            self.motorBackLeft.spin(FORWARD, tempYVel - tempXVel + rotVel * rotationFactor)
            self.motorBackRight.spin(FORWARD, tempYVel + tempXVel - rotVel * rotationFactor)
            
            wait(20)  # Small delay for control loop

        # We've reached our destination or been interrupted
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
            # orange closest
            objectToCenter = orangeClosest
        elif (greenClosest.width > orangeClosest.width and greenClosest.width > yellowClosest.width):
            # green closest
            objectToCenter = greenClosest
        elif (yellowClosest.width > orangeClosest.width and yellowClosest.width > greenClosest.width):
            # yellow closest
            objectToCenter = yellowClosest
        else:
            # no object found
            pass

        if (objectToCenter is not None):
            # Calculate the center of the object
            objectCenterX = objectToCenter.centerX - cameraXOffset
            objectCenterY = objectToCenter.centerY - cameraYOffset

            # Calculate the angle to turn to center the object
            angleToTurn = math.atan2(objectCenterY, objectCenterX) * (180 / math.pi)

            # Rotate the robot to face the object
            self.driveToPose(self.x, self.y, angleToTurn)