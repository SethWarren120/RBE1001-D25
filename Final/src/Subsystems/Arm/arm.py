from vex import *
from constants import *

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

            # PID variables
            kp, ki, kd = armPID
            integral = 0
            prevError = 0

            # while True:
            # Calculate the error
            currentLength = self.getLength()
            error = targetLength - currentLength

            # Break the loop if the error is within a small tolerance
            # if abs(error) < armTolerance:
            #     break

            # PID calculations
            integral += error
            derivative = error - prevError
            output = kp * error + ki * integral + kd * derivative

            # Spin the motor based on the PID output
            self.armmotorR.spin(FORWARD, output, PERCENT)
            self.armmotorL.spin(FORWARD, output, PERCENT)

            # Update previous error
            prevError = error

            self.armLength = self.armmotorL.position(DEGREES) / armGearRatio

            # Stop the motor once the target is reached
            # self.armmotorL.stop()
            # self.armmotorR.stop()

    def getAngle(self):
        self.armAngle = self.pivotmotor.position(DEGREES) / pivotGearRatio
        return self.armAngle
    
    def setAngle(self):
        while True:
            targetAngle = self.clamp(self.dAngle, minArmAngle, maxArmAngle)

            # PID variables
            kp, ki, kd = pivotPID
            integral = 0
            prevError = 0

            # while True:
            # Calculate the error
            currentAngle = self.getAngle()
            error = targetAngle - currentAngle

            # Break the loop if the error is within a small tolerance
            # if abs(error) < pivotTolerance:
            #     break

            # PID calculations
            integral += error
            derivative = error - prevError
            output = (kp * error + pivotFF*math.cos(math.radians(self.getAngle()))) + ki * integral + kd * derivative
            # Spin the motor based on the PID output
            self.pivotmotor.spin(FORWARD, output, PERCENT)

            # Update previous error
            prevError = error
            
            self.armAngle = self.pivotmotor.position(DEGREES) / pivotGearRatio

            # Stop the motor once the target is reached
            # self.pivotmotorL.stop()
            # self.pivotmotorR.stop()
    
    def getWristAngle(self):
        self.wristAngle = self.wristmotor.position(DEGREES) / wristGearRatio
        return self.wristmotor.position(DEGREES) / wristGearRatio
    
    def setWristAngle(self):
        while True:
            targetAngle = self.clamp(self.dWrist, minWristAngle, maxWristAngle)

            # PID variables
            kp, ki, kd = wristPID
            integral = 0
            prevError = 0

            # while True:
            # Calculate the error
            currentAngle = self.getWristAngle()
            error = targetAngle - currentAngle

            # Break the loop if the error is within a small tolerance
            # if abs(error) < wristTolerance:
            #     break

            # PID calculations
            integral += error
            derivative = error - prevError
            output = kp * error + ki * integral + kd * derivative

            # Spin the motor based on the PID output
            self.wristmotor.spin(FORWARD, output, PERCENT)

            # Update previous error
            prevError = error
            
            self.wristAngle = self.wristmotor.position(DEGREES) / wristGearRatio

            # Stop the motor once the target is reached
            # self.wristmotor.stop()

    def clamp(self, value, min_value, max_value):
        return max(min_value, min(value, max_value))
    
    def setSetpoint(self, length, angle, wristAngle):
        self.dLength = length
        self.dAngle = angle
        self.dWrist = wristAngle