from vex import *
from constants import *

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

        # PID variables
        kp, ki, kd = armPID
        integral = 0
        prevError = 0

        while True:
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
        self.armmotorL.stop()
        self.armmotorR.stop()


    def setAngle(self):
        targetAngle = self.clamp(self.dAngle, minArmAngle, maxArmAngle)

        # PID variables
        kp, ki, kd = pivotPID
        integral = 0
        prevError = 0

        while True:
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
            self.pivotmotorL.spin(FORWARD, output, PERCENT)
            self.pivotmotorR.spin(FORWARD, output, PERCENT)

            # Update previous error
            prevError = error
            
            self.armAngle = self.pivotmotorL.position(DEGREES) / pivotGearRatio

        # Stop the motor once the target is reached
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

        # PID variables
        kp, ki, kd = wristPID
        integral = 0
        prevError = 0

        while True:
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
        # zeroAngleThread = Thread(lambda: self.zeroAngle())
        # zeroLengthThread = Thread(lambda: self.zeroLength())
        # self.zeroWrist()
        self.zeroAngle()

    def clamp(self, value, min_value, max_value):
        return max(min_value, min(value, max_value))
    
    def setSetpoint(self, length, angle, wristAngle):
        self.dLength = length
        self.dAngle = angle
        self.dWrist = wristAngle