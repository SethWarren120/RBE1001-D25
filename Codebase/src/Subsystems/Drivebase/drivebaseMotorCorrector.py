# Library imports
from vex import *
from asyncio import create_task

class DrivebaseMotorCorrector:

    motors = []

    offset = 1
    offsetList = []
    tolerance = 5
    units = DEGREES

    passive = False
    motorSnapshot = []
    loopTime = 20

    enabled = True
    configured = True

    def __init__(self, motorList, config):
        if len(motorList) != len(config.offsetList):
            raise Exception("Starting offset list does not contain the same number of entries as motors")
        
        self.motors = motorList
        self.offset = config.offset
        self.offsetList = config.offsetList
        self.tolerance = config.tolerance
        self.units = config.rotationUnits
        self.loopTime = config.loopTime
        self.enabled = config.enabled
        self.configured = config.configured

    def setEnabled(self, enabledState, motorOffsetList = None):
        if not self.configured:
            raise Exception("Cannot enable Motor Correction unless it is configured")

        self.enabled = enabledState

        if motorOffsetList == None:
            self.offsetList = [0] * len(self.motors)
            self.correctMotors([1] * len(self.motors))
        else:
            self.offsetList = motorOffsetList

    def setPassiveMode(self, usePassiveMode):
        if not self.enabled:
            return
        
        self.passive = usePassiveMode
        if (usePassiveMode and ((not hasattr(self, 'passiveCorrectionTask')) or
                                     (hasattr(self, 'passiveCorrectionTask') & self.passiveCorrectionTask.done()))):
            self.motorSnapshot = self.__getMotorSnapshot()
            self.passiveCorrectionTask = create_task(self.__passiveCorrectionLoop())
        elif not usePassiveMode:
            while hasattr(self, 'passiveCorrectionTask') and not self.passiveCorrectionTask.done():
                sleep(self.loopTime / 4)

    # Manual correction mode
    def correctMotors(self, *travelDirections):
        if not self.enabled:
            return
        
        if len(travelDirections) != len(self.motors):
            raise Exception("Travel direction list does not contain the same number of entries as motors")
        
        if self.passive:
            raise Exception("Cannot use Manual Correction Mode while Self Correction Mode is active")

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

    async def __passiveCorrectionLoop(self):
        while self.passive:
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

        return self.motors[motorIndex].get_position(rotationUnits) - self.offsetList[motorIndex] if self.enabled else 0
    
class DrivebaseMotorCorrectionProfile:

    offset = 1
    startingOffsets = []
    tolerance = 5
    passive = False
    units = DEGREES
    loopTime = 20
    enabled = True
    configured = True

    def __init__(self, offsetAmount, startingOffsetList, correctionTolerance, passiveMode = False,
                 correctionEnabled = True, rotationUnits = DEGREES, correctionLoopTime = 20):
        self.offset = offsetAmount
        self.startingOffsets = startingOffsetList
        self.tolerance = correctionTolerance
        self.passive = passiveMode
        self.units = rotationUnits
        self.loopTime = correctionLoopTime
        self.enabled = correctionEnabled
        self.configured = True

    @staticmethod
    def Disabled(rotationUnits = DEGREES):
        config = DrivebaseMotorCorrectionProfile(0, [], 0, False, False, rotationUnits)
        config.configured = False
        return config