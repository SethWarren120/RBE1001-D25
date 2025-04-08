from vex import *

class Subsystem():
    def __init__(self):
        self.defaultCommand = None
        self.currentCommand = None

        self.periodic()
        
    def setDefaultCommand(self, command):
        self.defaultCommand = command

    def run(self, command):
        self.currentCommand = command
        runThread = Thread(lambda: command)

    def periodic(self):
        while True:
            if (self.currentCommand == None):
                self.run(self.defaultCommand)
            wait(20)