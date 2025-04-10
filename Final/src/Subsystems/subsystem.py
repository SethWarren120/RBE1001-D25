from vex import *

class Subsystem():
    def __init__(self):
        self.defaultCommand = None
        self.currentCommand = None
        self.commandList = []
        periodicThread  = Thread(self.periodic)

    def setDefaultCommand(self, command: function):
        self.defaultCommand = command

    def run(self, command, priority: bool):
        if priority:
            self.commandList.insert(0, command)
        else:
            self.commandList.append(command)

    def periodic(self):
        while True:
            if (self.commandList.count == 0):
                if (self.defaultCommand != None):
                    self.commandList.append(self.defaultCommand)
            
            self.currentCommand = self.commandList.pop(0)
            self.currentCommand()
            
            wait(20)