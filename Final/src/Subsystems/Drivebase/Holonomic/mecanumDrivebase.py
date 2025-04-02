# Library imports
from vex import *
from Subsystems.Drivebase.drivebaseMotorCorrector import *
import math


class MecanumDrivebase ():

    diameter = 4
    gearing = 5 / 1
    wheelBase = 11
    circumference = math.pi * diameter
    
    def __init__(self, _motorFrontLeft, _motorFrontRight, _motorBackLeft, _motorBackRight, _gyro, drivePID, turnPID, wheelDiameter, gearRatio, drivebaseWidth,
                 motorCorrectionConfig = None, _rotationUnits = DEGREES, _speedUnits = RPM):
        self.motorFrontLeft = _motorFrontLeft
        self.motorFrontRight = _motorFrontRight
        self.motorBackLeft = _motorBackLeft
        self.motorBackRight = _motorBackRight
        self.gyro = _gyro

        self.diameter = wheelDiameter
        self.gearing = gearRatio
        self.wheelBase = drivebaseWidth
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

        if motorCorrectionConfig == None:
            motorCorrectionConfig = DrivebaseMotorCorrectionProfile.Disabled(_rotationUnits)
        else:
            motorCorrectionConfig.units = _rotationUnits

        odometryThread = Thread(self.updateOdometry)
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