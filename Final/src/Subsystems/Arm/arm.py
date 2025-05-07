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
        leftLength = self.armmotorL.position(DEGREES) / armGearRatio
        rightLength = self.armmotorR.position(DEGREES) / armGearRatio
        self.armLength = (leftLength+rightLength)/2
        return self.armLength

    def setLength(self):
        while True:
            targetLength = self.clamp(self.dLength, minArmLength, maxArmLength)

            if (abs(self.getAngle()-self.dAngle) > pivotTolerance*5):
                targetLength = self.getLength()

            kp, ki, kd = armPID
            integral = 0
            prevError = 0

            currentLength = self.getLength()
            error = targetLength - currentLength

            integral += error
            derivative = error - prevError
            output = kp * error + ki * integral + kd * derivative

            if (self.armmotorL.torque() > 1.5 or self.armmotorL.torque() > 1.5 or (self.getLength() == 0 and targetLength == 0)):
                self.armmotorR.stop()
                self.armmotorL.stop()
                self.armmotorR.set_position(0, DEGREES)
                self.armmotorL.set_position(0, DEGREES)
            else:
                self.armmotorL.spin(FORWARD, output, PERCENT)
                self.armmotorR.spin(FORWARD, output, PERCENT)

            prevError = error

            self.armLength = self.armmotorL.position(DEGREES) / armGearRatio

    def getAngle(self):
        leftAngle = self.pivotmotorL.position(DEGREES) / pivotGearRatio
        rightAngle = self.pivotmotorR.position(DEGREES) / pivotGearRatio
        self.armAngle = (leftAngle+rightAngle)/2
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
            
            clampedOutput = self.clamp(output, -pivotMaxSpeed, pivotMaxSpeed)
            if (self.pivotmotorL.torque() > 1.5 or self.pivotmotorR.torque() > 1.5 or (self.getAngle() == 0 and targetAngle == 0)):
                self.pivotmotorR.stop()
                self.pivotmotorL.stop()
                self.pivotmotorR.set_position(0, DEGREES)
                self.pivotmotorL.set_position(0, DEGREES)
            else:
                self.pivotmotorL.spin(FORWARD, clampedOutput, PERCENT)
                self.pivotmotorR.spin(FORWARD, clampedOutput, PERCENT)

            prevError = error
            
            self.armAngle = self.pivotmotorL.position(DEGREES) / pivotGearRatio
    
    def getWristAngle(self):
        self.wristAngle = self.wristmotor.position(DEGREES) / wristGearRatio
        return self.wristAngle
    
    def setWristAngle(self):
        while True:
            targetAngle = self.dWrist
            if targetAngle > 180:
                targetAngle -= 360
            
            if (abs(self.getLength()-self.dLength) > armTolerance*10 and self.dLength > 130):
                targetAngle = self.getWristAngle()

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

    def flipWrist(self):
        self.setSetpoint(self.dLength, self.dAngle, self.dWrist+180)

    def clamp(self, value, min_value, max_value):
        return max(min_value, min(value, max_value))
    
    def setSetpoint(self, length, angle, wristAngle):
        self.dAngle = angle
        self.dLength = length
        self.dWrist = wristAngle

    def stowArm(self):
        currentWristAngle = self.getWristAngle()
        print(currentWristAngle)
        if abs(currentWristAngle - 0) < abs(currentWristAngle - 180):
            if abs(currentWristAngle - 0) < abs(currentWristAngle - 180) and abs(currentWristAngle - 0) < abs(currentWristAngle + 180):
                self.setSetpoint(self.getLength(), self.getAngle(), 0)
            elif abs(currentWristAngle - 180) < abs(currentWristAngle + 180):
                self.setSetpoint(self.getLength(), self.getAngle(), 180)
            else:
                self.setSetpoint(self.getLength(), self.getAngle(), -180)

        while(abs(self.getWristAngle()-self.dWrist) > wristTolerance):
            sleep(1)

        self.setSetpoint(0, self.getAngle(), self.getWristAngle())

        while(abs(self.getLength()-self.dLength) > armTolerance):
            sleep(1)

        self.setSetpoint(0, 0, self.getWristAngle())

    def atSetpoint(self):
        if (abs(self.getLength()-self.dLength) < armTolerance and abs(self.getAngle()-self.dAngle) < pivotTolerance and abs(self.getWristAngle()-self.dWrist) < wristTolerance):
            return True
        else:
            return False