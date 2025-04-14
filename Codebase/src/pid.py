from vex import *

class Pid:

    p = 1
    i = 0
    d = 0.1

    errorLast = 0
    sampleTime = 0
    errorSum = 0
    sampleCount = 0

    target = 0

    timer = Timer()

    def __init__(self, kP, kI, kD):
        self.p = kP
        self.i = kI
        self.d = kD

    def setTarget(self, setpoint):
        self.target = setpoint

    def update(self, position, setpoint = target):
        time = self.timer.time(TimeUnits.SEC)
        error = setpoint - position
        self.errorSum += error
        self.sampleCount += 1
        errorAvg = self.errorSum / self.sampleCount
        errorSlope = (error - self.errorLast) / (time - self.sampleTime)

        self.errorLast = error
        self.sampleTime = time

        return self.p * error + self.i * errorAvg + self.d * errorSlope
    
    def resetIntetgral(self):
        self.errorSum = 0
        self.sampleCount = 0