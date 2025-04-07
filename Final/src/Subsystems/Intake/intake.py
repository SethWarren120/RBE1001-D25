from vex import *

class Intake ():
    def __init__(self, intakemotor):
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
        self.runIntake("reverse")
        while self.intakemotor.current < 0.5 or self.escapeLoop == False:
            pass
        self.stopIntake()
