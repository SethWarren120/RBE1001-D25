from vex import *
from Subsystems.subsystem import *

class Intake (Subsystem):
    def __init__(self, intakemotor: Motor):
        self.intakemotor = intakemotor

        self.escapeLoop = False

    def runIntake(self, direction):
        if direction == "forward":
            self.intakemotor.spin(FORWARD)
        else:
            self.intakemotor.spin(REVERSE)

    def stopIntake(self):
        self.escapeLoop = True
        self.intakemotor.stop()
        self.escapeLoop = False

    def intakeUntilCurrent(self):
        while self.intakemotor.current() < 0.5 or self.escapeLoop == False:
            self.runIntake("reverse")
        self.stopIntake()

    def intakeUntilCurrentCommand(self):
        self.run(self.intakeUntilCurrent())
        self.currentCommand = None

    def stopIntakeCommand(self):
        self.run(self.stopIntake())
        self.currentCommand = None