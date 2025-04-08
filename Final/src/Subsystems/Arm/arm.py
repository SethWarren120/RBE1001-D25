from vex import *
from constants import *
from Subsystems.subsystem import *

class Arm (Subsystem):
    def __init__(self, armmotor, pivotmotor, wristmotor):
        self.armmotor = armmotor
        self.pivotmotor = pivotmotor
        self.wristmotor = wristmotor

        self.armLength = 0
        self.armAngle = 0
        self.wristAngle = 0

    def getLength(self):
        return self.armLength

    def setLength(self, length):
        actualLength = self.clamp(length, minArmLength, maxArmLength)
        targetLength = actualLength

        # PID variables
        kp, ki, kd = armPID
        integral = 0
        prevError = 0

        while True:
            # Calculate the error
            currentLength = self.getLength()
            error = targetLength - currentLength

            # Break the loop if the error is within a small tolerance
            if abs(error) < armTolerance:
                break

            # PID calculations
            integral += error
            derivative = error - prevError
            output = kp * error + ki * integral + kd * derivative

            # Spin the motor based on the PID output
            self.armmotor.spin(FORWARD, output, PERCENT)

            # Update previous error
            prevError = error

        # Stop the motor once the target is reached
        self.armmotor.stop()
        self.armLength = self.armmotor.position(DEGREES) / armGearRatio

    def setAngle(self, angle):
        actualAngle = self.clamp(angle, minArmAngle, maxArmAngle)
        targetAngle = actualAngle

        # PID variables
        kp, ki, kd = pivotPID
        integral = 0
        prevError = 0

        while True:
            # Calculate the error
            currentAngle = self.getAngle()
            error = targetAngle - currentAngle

            # Break the loop if the error is within a small tolerance
            if abs(error) < pivotTolerance:
                break

            # PID calculations
            integral += error
            derivative = error - prevError
            output = kp * error + ki * integral + kd * derivative

            # Spin the motor based on the PID output
            self.pivotmotor.spin(FORWARD, output, PERCENT)

            # Update previous error
            prevError = error

        # Stop the motor once the target is reached
        self.pivotmotor.stop()
        self.armAngle = self.pivotmotor.position(DEGREES) / wristGearRatio
    
    def getAngle(self):
        return self.armAngle
    
    def getWristAngle(self):
        return self.wristmotor.position(DEGREES) / wristGearRatio
    
    def setWristAngle(self, angle):
        actualAngle = self.clamp(angle, minWristAngle, maxWristAngle)
        targetAngle = actualAngle

        # PID variables
        kp, ki, kd = wristPID
        integral = 0
        prevError = 0

        while True:
            # Calculate the error
            currentAngle = self.getWristAngle()
            error = targetAngle - currentAngle

            # Break the loop if the error is within a small tolerance
            if abs(error) < wristTolerance:
                break

            # PID calculations
            integral += error
            derivative = error - prevError
            output = kp * error + ki * integral + kd * derivative

            # Spin the motor based on the PID output
            self.wristmotor.spin(FORWARD, output, PERCENT)

            # Update previous error
            prevError = error

        # Stop the motor once the target is reached
        self.wristmotor.stop()
        self.wristAngle = self.wristmotor.position(DEGREES) / wristGearRatio

    def toPositionCommand(self, length, angle, wristAngle):
        armLengthThread = Thread(lambda: self.setLength(length))
        armAngleThead = Thread(lambda: self.setAngle(angle))
        self.setWristAngle(wristAngle)
        self.currentCommand = None

    def clamp(self, value, min_value, max_value):
        return max(min_value, min(value, max_value))