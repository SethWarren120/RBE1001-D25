from vex import *

class Forks ():
    def __init__(self, forkmotor, rollerMotor):
        self.forkmotor = forkmotor
        self.rollerMotor = rollerMotor

        self.forksDeployed = False

    def deployForks(self):
        self.forkmotor.spin_for(FORWARD, 90, DEGREES)
        self.forksDeployed = True

    def retractForks(self):
        self.forkmotor.spin_for(FORWARD, 0, DEGREES)
        self.forksDeployed = False

    def rollDirection(self, direction, distance):
        #calculate the number of rotations based on the distance and roller diameter
        if direction == "left":
            self.rollerMotor.spin_for(FORWARD, distance)
        elif direction == "right":
            self.rollerMotor.spin_for(REVERSE, distance)

    def toggleForks(self):
        if self.forksDeployed:
            self.retractForks()
        else:
            self.deployForks()