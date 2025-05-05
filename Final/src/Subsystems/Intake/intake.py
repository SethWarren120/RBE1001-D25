from vex import *

class Intake ():
    def __init__(self, intakemotor: Motor):
        self.intakemotor = intakemotor
        self.intakemotor.set_velocity(100, PERCENT)

        self.escapeLoop = False
        
    def runIntake(self, direction):
        if direction == FORWARD:
            self.intakemotor.spin(FORWARD)
        else:
            self.intakemotor.spin(REVERSE)

    def stopIntake(self):
        self.escapeLoop = True
        self.intakemotor.stop()
        self.escapeLoop = False

    def runTimeThread(self, direction, time):
        self.runIntake(direction)
        sleep(time)
        self.stopIntake()

    def runIntakeForTime(self, direction, time):
        intakeTimeThread = Thread(lambda: self.runTimeThread(direction, time))