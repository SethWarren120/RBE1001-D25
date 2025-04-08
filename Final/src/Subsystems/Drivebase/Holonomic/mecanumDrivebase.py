# Library imports
from vex import *
from Subsystems.Drivebase.drivebaseMotorCorrector import *
import math
from constants import *
from Subsystems.subsystem import *

class MecanumDrivebase (Subsystem):
    def __init__(self, _motorFrontLeft: Motor, _motorFrontRight: Motor, _motorBackLeft: Motor, _motorBackRight: Motor, 
                 _gyro: Inertial, _camera: Vision, _rotationUnits = DEGREES, _speedUnits = RPM):
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
        findObjectsThread = Thread(self.calculateObjectPostion)

    def drive(self, xVel, yVel, rotVel):
        rotationFactor = (wheel_base + track_width) / 2.0

        # Get the current heading from the gyro
        heading = self.gyro.heading(DEGREES)
        headingRadians = math.radians(heading)  # Convert heading to radians

        # Apply field-centric transformation
        tempXVel = xVel * math.cos(headingRadians) - yVel * math.sin(headingRadians)
        tempYVel = xVel * math.sin(headingRadians) + yVel * math.cos(headingRadians)

        # PID for linear velocity
        distanceError = math.sqrt(tempXVel**2 + tempYVel**2)
        integralDistanceError = 0
        derivativeDistanceError = 0
        prevDistanceError = 0

        integralDistanceError += distanceError
        derivativeDistanceError = distanceError - prevDistanceError
        linearOutput = (
            drivePID[0] * distanceError +
            drivePID[1] * integralDistanceError +
            drivePID[2] * derivativeDistanceError
        )
        prevDistanceError = distanceError

        # Scale the velocities using linearOutput
        scaledXVel = linearOutput * (tempXVel / distanceError) if distanceError != 0 else 0
        scaledYVel = linearOutput * (tempYVel / distanceError) if distanceError != 0 else 0

        # PID for rotational velocity
        headingError = rotVel - self.heading
        headingError = (headingError + 180) % 360 - 180  # Normalize to [-180, 180]
        integralHeadingError = 0
        derivativeHeadingError = 0
        prevHeadingError = 0

        integralHeadingError += headingError
        derivativeHeadingError = headingError - prevHeadingError
        rotationalOutput = (
            turnPID[0] * headingError +
            turnPID[1] * integralHeadingError +
            turnPID[2] * derivativeHeadingError
        )
        prevHeadingError = headingError

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
    
    def calculateObjectPostion(self):
        self.camera.take_snapshot(0)

        largestObject = self.camera.largest_object()
        if largestObject.exists:
            # Calculate the distance to the fruit using the ratio of actual width to image width
            if largestObject.width > 0:

                distanceToSmallFruit = (smallFruitWidth / largestObject.width) * cameraFocalLength
                distanceToLargeFruit = (largeFruitWidth / largestObject.width) * cameraFocalLength

                # Calculate the angle to the fruit relative to the robot's heading
                angleToFruit = math.radians(largestObject.centerX - (largestObject.width / 2)) * cameraFOV

                # Calculate the fruit's position in the field
                smallFruitX = self.x + distanceToSmallFruit * math.cos(math.radians(self.heading) + angleToFruit)
                smallFruitY = self.y + distanceToSmallFruit * math.sin(math.radians(self.heading) + angleToFruit)
                largeFruitX = self.x + distanceToLargeFruit * math.cos(math.radians(self.heading) + angleToFruit)
                largeFruitY = self.y + distanceToLargeFruit * math.sin(math.radians(self.heading) + angleToFruit)

                # Store the fruit's position
                self.objectLocations.append((("small", smallFruitX, smallFruitY), ("large", largeFruitX, largeFruitY)))
                self.removeRedundant()

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
    
    def zeroGyro(self):
        self.gyro.set_heading(0, DEGREES)

    def driveCommand(self, x, y, theta):
        self.run(self.drive(x,y,theta))
        self.currentCommand = None

    def driveToPoseCommand(self, x, y, theta):
        self.run(self.driveToPose(x,y,theta))
        self.currentCommand = None

    def zeroGyroCommand(self):
        self.run(self.zeroGyro())
        self.currentCommand = None