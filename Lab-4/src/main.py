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

fl_motor = Motor(Ports.PORT1, 18_1, True)
fr_motor = Motor(Ports.PORT9, 18_1, False)
bl_motor = Motor(Ports.PORT2, 18_1, True)
br_motor = Motor(Ports.PORT10, 18_1, False)

inertial = Inertial(Ports.PORT3)

camera = Vision(Ports.PORT4, 50)

drivebase = TankDrivebase(fl_motor, fr_motor, inertial, camera)

setSubsystems(drivebase)

controller = Controller()

drivebase.setDefaultCommand(lambda: drivebase.driveCommand(controller.axis3, controller.axis2))