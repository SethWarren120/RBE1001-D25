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

arm_motorL = Motor(Ports.PORT14, 18_1, False)
arm_motorR = Motor(Ports.PORT15, 18_1, True)
wrist_motor = Motor(Ports.PORT11, 18_1, False)
pivot_motorL = Motor(Ports.PORT17, 18_1, True)
pivot_motorR = Motor(Ports.PORT16, 18_1, False)
intake_motor = Motor(Ports.PORT5, 18_1, False)
fork_motor = Motor(Ports.PORT8, 18_1, False)

inertial = Inertial(Ports.PORT3)

camera = AiVision(Ports.PORT4, 50)
camera.start_awb()

# drivebase = MecanumDrivebase(fl_motor, fr_motor, bl_motor, br_motor, inertial, camera)
# intake = Intake(intake_motor)
# forks = Forks(fork_motor)
arm = Arm(arm_motorL, arm_motorR, pivot_motorL, pivot_motorR, wrist_motor)

# setSubsystems(drivebase, arm, intake, forks)

controller = Controller()

def printDebugging():
    while True:
        brain.screen.print_at(arm.getLength(), x=1, y=60)
        brain.screen.print_at(arm.getAngle(), x=1, y=80)
        brain.screen.print_at(arm.getWristAngle(), x=1, y=100)
        sleep(100)


debugThread = Thread(printDebugging)
# arm.zeroing()

arm.setSetpoint(100, 90, 0)

# controller.buttonX.pressed(lambda: arm.setSetpoint(0, 50, 0))
# controller.buttonY.pressed(lambda: arm.setSetpoint(0, 0, 0))

# while True:
#     driveSub.drive(controller.axis4.position(), controller.axis3.position(), controller.axis1.position())