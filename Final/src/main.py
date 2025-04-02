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


from Subsystems.Drivebase.Holonomic.mecanumDrivebase import MecanumDrivebase
from Subsystems.Drivebase.drivebaseMotorCorrector import *

#Constants
wheelDiameter = 4.0
wheel_travel = math.pi*wheelDiameter
track_width = 11
wheel_base = 11
gear_ratio = 5
wheelCircumference = 3.14 * wheelDiameter 
degreesPerInch = 360.0 / wheelCircumference

brain=Brain()

fl_motor = Motor(Ports.PORT1, 18_1, True)
fr_motor = Motor(Ports.PORT9, 18_1, False)
bl_motor = Motor(Ports.PORT2, 18_1, True)
br_motor = Motor(Ports.PORT10, 18_1, False)

inertial = Inertial(Ports.PORT3)

drivePID = [1,0,0]
turnPID = [1,0,0]

drivebase = MecanumDrivebase(fl_motor, fr_motor, bl_motor, br_motor, inertial, drivePID, turnPID, wheelDiameter, gear_ratio, wheel_base)

controller = Controller()

controller.buttonA.pressed(lambda: drivebase.driveToPose(0, 0, 0))
controller.buttonB.pressed(lambda: drivebase.driveToPose(0, 0, 90))

drivebase.drive(controller.axis1, controller.axis2, controller.axis4)