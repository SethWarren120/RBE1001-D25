# Library imports
from vex import *
from Subsystems.Drivebase.drivebaseMotorCorrector import *
import math
from constants import *

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
        # self.motorCorrector = DrivebaseMotorCorrector([_motorLeft, _motorRight], motorCorrectionConfig)

    def drive(self, xVel, yVel, rotVel, durationSeconds=-1):
        startTime = time.time()  # Record the start time

        while durationSeconds == -1 or (time.time() - startTime < durationSeconds):
            # Get the current heading from the gyro
            heading = self.gyro.heading(DEGREES)
            headingRadians = math.radians(heading)  # Convert heading to radians

            # Apply field-centric transformation
            tempXVel = xVel * math.cos(headingRadians) - yVel * math.sin(headingRadians)
            tempYVel = xVel * math.sin(headingRadians) + yVel * math.cos(headingRadians)

            # Drive the motors with the transformed velocities
            self.motorFrontLeft.spin(FORWARD, tempYVel + tempXVel + rotVel)
            self.motorFrontRight.spin(FORWARD, tempYVel - tempXVel - rotVel)
            self.motorBackLeft.spin(FORWARD, tempYVel - tempXVel + rotVel)
            self.motorBackRight.spin(FORWARD, tempYVel + tempXVel - rotVel)

        # Stop the motors after the loop ends
        self.motorFrontLeft.stop()
        self.motorFrontRight.stop()
        self.motorBackLeft.stop()
        self.motorBackRight.stop()

    def updateOdometry(self):
        frontLeftDistance = self.motorFrontLeft.position(DEGREES) / 360.0 * self.circumference
        frontRightDistance = self.motorFrontRight.position(DEGREES) / 360.0 * self.circumference
        backLeftDistance = self.motorBackLeft.position(DEGREES) / 360.0 * self.circumference
        backRightDistance = self.motorBackRight.position(DEGREES) / 360.0 * self.circumference

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

    def driveToPose(self, x, y, heading, tolerance=0.5):
        self.driveDuration = 0

        prevDistanceError = 0
        prevHeadingError = 0
        integralDistanceError = 0
        integralHeadingError = 0

        while True:
            # Calculate the error in position and heading
            deltaX = x - self.x
            deltaY = y - self.y
            distanceError = math.sqrt(deltaX**2 + deltaY**2)
            headingError = heading - self.heading

            # Normalize heading error to the range [-180, 180]
            headingError = (headingError + 180) % 360 - 180

            # Check if the robot is within the tolerances
            if distanceError <= tolerance and abs(headingError) <= tolerance:
                break

            # PID for position
            integralDistanceError += distanceError
            derivativeDistanceError = distanceError - prevDistanceError
            positionOutput = (
                self.drivePID[0] * distanceError +
                self.drivePID[1] * integralDistanceError +
                self.drivePID[2] * derivativeDistanceError
            )
            prevDistanceError = distanceError

            # PID for heading
            integralHeadingError += headingError
            derivativeHeadingError = headingError - prevHeadingError
            headingOutput = (
                self.turnPID[0] * headingError +
                self.turnPID[1] * integralHeadingError +
                self.turnPID[2] * derivativeHeadingError
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
            self.drive(xVel, yVel, rotVel)

            # Update odometry
            self.updateOdometry()

        # Stop the robot once the target pose is reached
        self.drive(0, 0, 0)
    
    def calculateObjectPostion(self):
        self.camera.takeSnapshot()

        largestObject = self.camera.largest_object
        if largestObject.exists:
            # Calculate the distance to the fruit using the ratio of actual width to image width
            if largestObject.width > 0:

                distanceToSmallFruit = (smallFruitWidth / largestObject.width) * self.camera.focal_length
                distanceToLargeFruit = (largeFruitWidth / largestObject.width) * self.camera.focal_length

                # Calculate the angle to the fruit relative to the robot's heading
                angleToFruit = math.radians(largestObject.centerX - (self.camera.width / 2)) * self.camera.field_of_view

                # Calculate the fruit's position in the field
                smallFruitX = self.x + distanceToSmallFruit * math.cos(math.radians(self.heading) + angleToFruit)
                smallFruitY = self.y + distanceToSmallFruit * math.sin(math.radians(self.heading) + angleToFruit)
                largeFruitX = self.x + distanceToLargeFruit * math.cos(math.radians(self.heading) + angleToFruit)
                largeFruitY = self.y + distanceToLargeFruit * math.sin(math.radians(self.heading) + angleToFruit)

                # Store the fruit's position
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
                # Calculate the distance between the current fruit and the unique fruit
                distance = math.sqrt((fruit[1] - uniqueFruit[1])**2 + (fruit[2] - uniqueFruit[2])**2)
                if distance < objectThreashold:
                    isDuplicate = True
                    break
            if not isDuplicate:
                uniqueLocations.append(fruit)

        # Update the objectLocations list with unique positions
        self.objectLocations = uniqueLocations