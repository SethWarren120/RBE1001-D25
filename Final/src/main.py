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
intake_motor = Motor(Ports.PORT5, 18_1, False)
wrist_motor = Motor(Ports.PORT3, 18_1, False)
pivot_motorL = Motor(Ports.PORT4, 18_1, True)
pivot_motorR = Motor(Ports.PORT9, 18_1, False)

inertial = Inertial(Ports.PORT1)
inertial.set_heading(180, DEGREES)
inertial.calibrate()

camera = AiVision(Ports.PORT2, vision_Yellow, vision_Green, vision_Orange, vision_Pink, vision_GreenBox, vision_YellowBox, vision_OrangeBox, AiVision.ALL_TAGS)
camera.start_awb()

controller = Controller()

drivebase = TankDrivebase(motorLeft, motorRight, inertial, camera, sideUltrasonic, controller)
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
    sleep(1)


def grabFruit():
    intake.runIntake(FORWARD)
    arm.setSetpoint(arm.dLength, arm.dAngle, arm.dWrist-5)


controller.buttonR2.pressed(grabFruit)
controller.buttonR2.released(intake.stopIntake)

controller.buttonL2.pressed(arm.flipWrist)

controller.buttonB.pressed(dumpObject)

controller.buttonUp.pressed(lambda: arm.setSetpoint(post4Height[0], post4Height[1], post4Height[2]))
controller.buttonDown.pressed(lambda: arm.setSetpoint(post2Height[0], post2Height[1], post2Height[2]))
controller.buttonLeft.pressed(lambda: arm.setSetpoint(post3Height[0], post3Height[1], post3Height[2]))
controller.buttonRight.pressed(lambda: arm.setSetpoint(post1Height[0], post1Height[1], post1Height[2]))

controller.buttonX.pressed(arm.stowArm)

controller.buttonY.pressed(lambda: arm.setSetpoint(180, 0, 90))

def Ramp():

    while inertial.orientation(ROLL) > -23:
        drivebase.drive(100, 0)
    
    counter = 1000

    while counter > 0:
        drivebase.drive(40, -(sideUltrasonic.distance(INCHES) - 3.5) * drivePID[0])
        if inertial.orientation(ROLL) > -5:
            counter -= 1
    pass

    drivebase.drive(0, 0)


Ramp()