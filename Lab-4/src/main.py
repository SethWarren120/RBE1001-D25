# ---------------------------------------------------------------------------- #
#                                                                              #
# 	Module:       main.py                                                      #
# 	Author:       Seth Warren, Logan Bachman                                                        #
# 	Created:      3/19/2025, 7:45:29 AM                                        #
# 	Description:  V5 project                                                   #
#                                                                              #
# ---------------------------------------------------------------------------- #

# Library imports
from vex import *

from Subsystems.Drivebase.Tank.tankDrivebase import TankDrivebase
from commands import *

brain=Brain()

l_motor = Motor(Ports.PORT1, 18_1, True)
r_motor = Motor(Ports.PORT10, 18_1, False)
inertial = Inertial(Ports.PORT3)
camera = AiVision(Ports.PORT9, vision_orange)

drivebase = TankDrivebase(l_motor, r_motor, inertial, camera)

controller = Controller()

setSubsystems(drivebase)

# drivebase.setDefaultCommand(lambda: drivebase.driveCommand(controller.axis3, controller.axis2))

def visionFunc():
    while True:
        brain.screen.print_at("running", x=0, y=80)
        objects = camera.take_snapshot(vision_orange)
        if len(objects) > 0:
            brain.screen.print_at("Object found", x=0, y=20)
            brain.screen.print_at("X: " + str(objects[0].centerX+cameraXOffset), x=0, y=40)
            brain.screen.print_at("Y: " + str(objects[0].centerY+cameraYOffset), x=0, y=60)
            brain.screen.render()
        
        wait(20)

# visionThread = Thread(visionFunc)

# drivebase.run(drivebase.centerToObject())
# drivebase.centerToObject()

drivebase.lab()