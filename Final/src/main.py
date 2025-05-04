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
from Subsystems.Arm.arm import Arm
from Subsystems.Intake.intake import Intake
from commands import *

brain=Brain()

motorLeft = Motor(Ports.PORT20, 18_1, False)
motorRight = Motor(Ports.PORT10, 18_1, True)

sideUltrasonic = Sonar(brain.three_wire_port.e)

arm_motorL = Motor(Ports.PORT17, 18_1, False)
arm_motorR = Motor(Ports.PORT7, 18_1, True)
intake_motor = Motor(Ports.PORT6, 18_1, False)
wrist_motor = Motor(Ports.PORT16, 18_1, False)
pivot_motorL = Motor(Ports.PORT18, 18_1, True)
pivot_motorR = Motor(Ports.PORT9, 18_1, False)

inertial = Inertial(Ports.PORT1)
inertial.set_heading(180, DEGREES)
inertial.calibrate()

camera = Vision(Ports.PORT6, 50)

tagCamera = AiVision(Ports.PORT2, 8)
tagCamera.start_awb()

controller = Controller()

drivebase = TankDrivebase(motorLeft, motorRight, inertial, tagCamera, camera, sideUltrasonic)
intake = Intake(intake_motor)
arm = Arm(arm_motorL, arm_motorR, pivot_motorL, pivot_motorR, wrist_motor)

setSubsystems(drivebase, arm, intake)

def printDebugging():
    while True:
        brain.screen.print_at(arm.getLength(), x=1, y=60)
        brain.screen.print_at(arm.getAngle(), x=1, y=80)
        brain.screen.print_at(arm.getWristAngle(), x=1, y=100)
        sleep(20)

debugThread = Thread(printDebugging)

while inertial.is_calibrating():
    sleep(10)
# drivebase.goUpRamp()

# drivebase.drive(50,0)
arm.setSetpoint(270, 46, 42)
intake.runIntake(FORWARD)
# wait(2000)
# arm.setSetpoint(0, 0, 0)
# drivebase.moveLen(100, 200)
# arm.setSetpoint(270, 40, 0)
# wait(500)
# stowArm()
# controller.buttonA.pressed(lambda: arm.setSetpoint(0, 50, 0))
# controller.buttonA.released(lambda: arm.setSetpoint(0, 0, 0))