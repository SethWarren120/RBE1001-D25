# Library imports
from vex import *
from Subsystems.Drivebase.drivebaseMotorCorrector import *
import math
from constants import *
from Subsystems.subsystem import *

class MecanumDrivebase (Subsystem):
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

        odometryThread = Thread(self.updateOdometry)
        # findObjectsThread = Thread(self.calculateObjectPostion)

    def drive(self, xVel, yVel, rotVel):
        rotationFactor = (wheel_base + track_width) / 2.0

        # Get the current heading from the gyro
        heading = self.gyro.heading(DEGREES)  # Returns [0, 360)
        headingRadians = math.radians(heading)  # Convert heading to radians

        # Apply field-centric transformation
        tempXVel = xVel * math.cos(headingRadians) - yVel * math.sin(headingRadians)
        tempYVel = xVel * math.sin(headingRadians) + yVel * math.cos(headingRadians)

        # PID for linear velocity
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

        # Scale the velocities using linearOutput
        scaledXVel = linearOutput * (tempXVel / distanceError) if distanceError != 0 else 0
        scaledYVel = linearOutput * (tempYVel / distanceError) if distanceError != 0 else 0

        # Ensure rotVel is in [0, 360) for consistent comparison
        rotVel = rotVel % 360
        
        # PID for rotational velocity - normalize error to [-180, 180]
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

        # Drive the motors with the PID outputs
        self.motorFrontLeft.spin(FORWARD, scaledYVel + scaledXVel + rotationalOutput * rotationFactor)
        self.motorFrontRight.spin(FORWARD, scaledYVel - scaledXVel - rotationalOutput * rotationFactor)
        self.motorBackLeft.spin(FORWARD, scaledYVel - scaledXVel + rotationalOutput * rotationFactor)
        self.motorBackRight.spin(FORWARD, scaledYVel + scaledXVel - rotationalOutput * rotationFactor)
        
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
        heading = heading % 360

        prevDistanceError = 0
        prevHeadingError = 0
        integralDistanceError = 0
        integralHeadingError = 0
        
        # Calculate path to avoid obstacles
        waypoints = self.checkPathAndAvoidObstacles(self.x, self.y, x, y)
        currentWaypoint = 0
        
        while currentWaypoint < len(waypoints):
            targetX, targetY = waypoints[currentWaypoint]
            
            while True:
                # Calculate the error in position and heading
                deltaX = targetX - self.x
                deltaY = targetY - self.y
                distanceError = math.sqrt(deltaX**2 + deltaY**2)
                
                # Use final heading for the last waypoint, otherwise face toward waypoint
                if currentWaypoint == len(waypoints) - 1:
                    targetHeading = heading
                else:
                    # Point toward the waypoint
                    targetHeading = math.degrees(math.atan2(deltaY, deltaX))
                
                headingError = targetHeading - self.heading
                headingError = (headingError + 180) % 360 - 180

                # Check if the robot is within the tolerances of the current waypoint
                waypointTolerance = tolerance if currentWaypoint == len(waypoints) - 1 else 2.0
                headingTolerance = tolerance if currentWaypoint == len(waypoints) - 1 else 10.0
                
                if distanceError <= waypointTolerance and abs(headingError) <= headingTolerance:
                    break

                # PID for position
                integralDistanceError += distanceError
                derivativeDistanceError = distanceError - prevDistanceError
                positionOutput = (
                    drivePID[0] * distanceError +
                    drivePID[1] * integralDistanceError +
                    drivePID[2] * derivativeDistanceError
                )
                prevDistanceError = distanceError

                # PID for heading
                integralHeadingError += headingError
                derivativeHeadingError = headingError - prevHeadingError
                headingOutput = (
                    turnPID[0] * headingError +
                    turnPID[1] * integralHeadingError +
                    turnPID[2] * derivativeHeadingError
                )
                prevHeadingError = headingError

                # Calculate the desired velocities
                angleToTarget = math.atan2(deltaY, deltaX)  # Angle to the target in radians
                xVel = positionOutput * math.cos(angleToTarget)  # Scale speed by angle
                yVel = positionOutput * math.sin(angleToTarget)  # Scale speed by angle
                rotVel = headingOutput  # Use heading PID output for rotation

                # Limit the velocities to the maximum speed
                xVel = max(-200, min(200, xVel))
                yVel = max(-200, min(200, yVel))
                rotVel = max(-200, min(200, rotVel))
                
                # Drive the robot
                self.driveCommand(xVel, yVel, rotVel)

                # Update odometry
                self.updateOdometry()
                
                # Small delay to prevent CPU hogging
                wait(20)
            
            # Move to the next waypoint
            currentWaypoint += 1
    
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
                # Cross product divided by line length gives perpendicular distance
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

    # def calculateObjectPostion(self):
    #     self.camera.take_snapshot(0)

    #     largestObject = self.camera.largest_object()
    #     if largestObject.exists:
    #         pass
        
    #     wait(20)

    # def removeRedundant(self):
    #     uniqueLocations = []
    #     for fruit in self.objectLocations:
    #         isDuplicate = False
    #         for uniqueFruit in uniqueLocations:
    #             # Calculate the distance between the current fruit and the unique fruit
    #             distance = math.sqrt((fruit[1] - uniqueFruit[1])**2 + (fruit[2] - uniqueFruit[2])**2)
    #             if distance < objectThreashold:
    #                 isDuplicate = True
    #                 break
    #         if not isDuplicate:
    #             uniqueLocations.append(fruit)

    #     # Update the objectLocations list with unique positions
    #     self.objectLocations = uniqueLocations
    
    def zeroGyro(self):
        self.gyro.set_heading(0, DEGREES)

    def driveCommand(self, x, y, theta):
        self.run(self.drive(x,y,theta), False)

    def driveToPoseCommand(self, x, y, theta):
        self.run(self.driveToPose(x,y,theta), False)

    def zeroGyroCommand(self):
        self.run(self.zeroGyro(), False)