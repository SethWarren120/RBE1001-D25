from vex import *
from Subsystems.subsystem import *

class Intake (Subsystem):
    def __init__(self, intakemotor):
        self.intakemotor = intakemotor

        self.escapeLoop = False

    def runIntakeCommand(self, direction):
        if direction == "forward":
            self.intakemotor.spin(FORWARD)
        else:
            self.intakemotor.spin(REVERSE)
        self.currentCommand = None

    def stopIntakeCommand(self):
        self.escapeLoop = True
        self.intakemotor.stop()
        self.escapeLoop = False
        self.currentCommand = None

    def intakeUntilCurrentCommand(self):
        while self.intakemotor.current < 0.5 or self.escapeLoop == False:
            self.runIntakeCommand("reverse")
        self.stopIntakeCommand()
        self.currentCommand = None