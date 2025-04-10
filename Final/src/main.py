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
from Subsystems.Arm.arm import Arm
from Subsystems.Intake.intake import Intake
from Subsystems.Forks.forks import Forks
from commands import *

brain=Brain()

fl_motor = Motor(Ports.PORT1, 18_1, True)
fr_motor = Motor(Ports.PORT9, 18_1, False)
bl_motor = Motor(Ports.PORT2, 18_1, True)
br_motor = Motor(Ports.PORT10, 18_1, False)

arm_motor = Motor(Ports.PORT6, 18_1, False)
wrist_motor = Motor(Ports.PORT11, 18_1, False)
pivot_motor = Motor(Ports.PORT7, 18_1, False)
intake_motor = Motor(Ports.PORT5, 18_1, False)
fork_motor = Motor(Ports.PORT8, 18_1, False)

inertial = Inertial(Ports.PORT3)

camera = Vision(Ports.PORT4, 50)

drivebase = MecanumDrivebase(fl_motor, fr_motor, bl_motor, br_motor, inertial, camera)
intake = Intake(intake_motor)
forks = Forks(fork_motor)
arm = Arm(arm_motor, pivot_motor, wrist_motor)

setSubsystems(drivebase, arm, intake, forks)

controller = Controller()

drivebase.setDefaultCommand(lambda: drivebase.driveCommand(controller.axis3, controller.axis4, controller.axis2))
arm.setDefaultCommand(lambda: arm.toPositionCommand(0, 0, 0))
intake.setDefaultCommand(lambda: intake.stopIntakeCommand())

controller.buttonA.pressed(lambda: drivebase.driveToPoseCommand(0, 0, 0))
controller.buttonB.pressed(lambda: drivebase.driveToPoseCommand(0, 0, 90))
controller.buttonX.pressed(lambda: drivebase.zeroGyroCommand())
# grabFruit((0, 0), lowFruitHeight)
# drivebase.drive(controller.axis3, controller.axis4, controller.axis2)