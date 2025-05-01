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
from commands import *

brain=Brain()

fl_motor = Motor(Ports.PORT10, 18_1, False)
fr_motor = Motor(Ports.PORT20, 18_1, True)
bl_motor = Motor(Ports.PORT1, 18_1, False)
br_motor = Motor(Ports.PORT11, 18_1, True)

arm_motorL = Motor(Ports.PORT14, 18_1, False)
arm_motorR = Motor(Ports.PORT15, 18_1, True)
wrist_motor = Motor(Ports.PORT13, 18_1, False)
pivot_motor = Motor(Ports.PORT12, 18_1, False)
intake_motor = Motor(Ports.PORT5, 18_1, False)

inertial = Inertial(Ports.PORT3)

camera = Vision(Ports.PORT6, 50)

tagCamera = AiVision(Ports.PORT4, 50)
tagCamera.start_awb()

controller = Controller()

drivebase = MecanumDrivebase(fl_motor, fr_motor, bl_motor, br_motor, inertial, tagCamera, camera, controller)
intake = Intake(intake_motor)
arm = Arm(arm_motorL, arm_motorR, pivot_motor, wrist_motor)

setSubsystems(drivebase, arm, intake)

def printDebugging():
    while True:
        brain.screen.print_at(arm.getLength(), x=1, y=60)
        brain.screen.print_at(arm.getAngle(), x=1, y=80)
        brain.screen.print_at(arm.getWristAngle(), x=1, y=100)
        sleep(20)


debugThread = Thread(printDebugging)


arm.setSetpoint(0, 10, 0)
# controller.buttonA.pressed(lambda: arm.setSetpoint(0, 50, 0))
# controller.buttonA.released(lambda: arm.setSetpoint(0, 0, 0))