from vex import *

class Intake ():
    def __init__(self, intakemotor):
        self.intakemotor = intakemotor

    def runIntake(self, direction):
        if direction == "forward":
            self.intakemotor.spin(FORWARD)
        else:
            self.intakemotor.spin(REVERSE)

    def stopIntake(self):
        self.intakemotor.stop()