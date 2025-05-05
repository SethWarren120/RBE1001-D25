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

            self.armmotorL.spin(FORWARD, output, PERCENT)
            self.armmotorR.spin(FORWARD, output, PERCENT)

            prevError = error

            self.armLength = self.armmotorL.position(DEGREES) / armGearRatio

    def getAngle(self):
        self.armAngle = self.pivotmotorL.position(DEGREES) / pivotGearRatio
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

            self.pivotmotorL.spin(FORWARD, output, PERCENT)
            self.pivotmotorR.spin(FORWARD, output, PERCENT)

            prevError = error
            
            self.armAngle = self.pivotmotorL.position(DEGREES) / pivotGearRatio
    
    def getWristAngle(self):
        self.wristAngle = self.wristmotor.position(DEGREES) / wristGearRatio
        return self.wristAngle
    
    def setWristAngle(self):
        while True:
            targetAngle = self.dWrist % 360
            if targetAngle > 180:
                targetAngle -= 360

            kp, ki, kd = wristPID
            integral = 0
            prevError = 0

            currentAngle = self.getWristAngle()
            error = targetAngle - currentAngle
            if error > 180:
                error -= 360
            elif error < -180:
                error += 360

            integral += error
            derivative = error - prevError
            output = kp * error + ki * integral + kd * derivative

            self.wristmotor.spin(FORWARD, output, PERCENT)

            prevError = error
            
            self.wristAngle = self.wristmotor.position(DEGREES) / wristGearRatio


    def clamp(self, value, min_value, max_value):
        return max(min_value, min(value, max_value))
    
    def setSetpoint(self, length, angle, wristAngle):
        self.dAngle = angle
        self.dLength = length
        self.dWrist = wristAngle

    def stowArm(self):
        if (abs(self.getWristAngle()) < abs(self.getWristAngle()-180)):
            self.setSetpoint(self.getLength(), self.getAngle(), 0)
        else:
            self.setSetpoint(self.getLength(), self.getAngle(), 180)

        while(abs(self.getWristAngle()-self.dWrist) > wristTolerance):
            sleep(1)

        self.setSetpoint(0, self.getAngle(), self.getWristAngle())

        while(abs(self.getLength()-self.dLength) > armTolerance):
            sleep(1)

        self.setSetpoint(0, 0, self.getWristAngle())