# Library imports
from vex import *
from asyncio import create_task

class DrivebaseMotorCorrector:

    motors = []

    offset = 1
    offsetList = []
    tolerance = 5
    units = DEGREES

    selfCorrectionMode = False
    motorSnapshot = []
    loopTime = 20

    def __init__(self, offsetAmount, startingOffsetList, motorList, correctionTolerance = 5, rotationUnits = DEGREES, 
                 correctionLoopTime = 20):
        if len(motorList) != len(startingOffsetList):
            raise Exception("Starting offset list does not contain the same number of entries as motors")
        
        self.motors = motorList
        self.offset = offsetAmount
        self.offsetList = startingOffsetList
        self.tolerance = correctionTolerance
        self.units = rotationUnits
        self.loopTime = correctionLoopTime

    def setSelfCorrectionMode(self, useSelfCorrectionMode):
        self.selfCorrectionMode = useSelfCorrectionMode
        if (useSelfCorrectionMode and ((not hasattr(self, 'selfCorrectionTask')) or
                                     (hasattr(self, 'selfCorrectionTask') & self.selfCorrectionTask.done()))):
            self.motorSnapshot = self.__getMotorSnapshot()
            self.selfCorrectionTask = create_task(self.__selfCorrectionLoop())

    # Manual correction mode
    def correctMotors(self, *travelDirections):
        if len(travelDirections) != len(self.motors):
            raise Exception("Travel direction list does not contain the same number of entries as motors")
        
        if self.selfCorrectionMode:
            raise Exception("Cannot use Manual Correction Mode while Self Correction Mode is active")

        while hasattr(self, 'selfCorrectionTask') and not self.selfCorrectionTask.done():
            sleep(self.loopTime / 4)

        for i in range(len(travelDirections)):
            if abs(self.offset - self.offsetList[i] if travelDirections[i] >= 0 else -self.offsetList[i]) < self.tolerance:
                continue

            self.motors[i].spin_for(FORWARD, self.offset - self.offsetList[i] if travelDirections[i] >= 0 else -self.offsetList[i], 
                                    self.units, 200, RPM, False)
            
        block = True

        while block:
            spinning = False
            for motor in self.motors:
                if motor.is_spinning():
                    spinning = True
            if spinning:
                sleep(self.loopTime)
            else:
                block = False

    async def __selfCorrectionLoop(self):
        while self.selfCorrectionMode:
            sleep(self.loopTime)
            snapshot = self.__getMotorSnapshot()
            for i in range(len(snapshot)):
                self.offsetList[i] = max(0, min(self.offset, self.offsetList[i] + (snapshot[i] - self.motorSnapshot[i])))
            self.motorSnapshot = snapshot

    def __getMotorSnapshot(self):
        snapshot = []
        for motor in self.motors:
            snapshot.append(motor.get_position())
        return snapshot
    
    def getPosition(self, motorIndex, rotationUnits = None):
        if rotationUnits == None:
            rotationUnits = self.units

        return self.motors[motorIndex].get_position(rotationUnits)