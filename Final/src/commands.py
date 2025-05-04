from vex import *
from constants import *
from Subsystems.Drivebase.Tank.tankDrivebase import TankDrivebase
from Subsystems.Arm.arm import Arm
from Subsystems.Intake.intake import Intake

def setSubsystems(driveBase: TankDrivebase, arm: Arm, intake: Intake):
    global driveSub, armSub, intakeSub, forksSub

    driveSub = driveBase
    armSub = arm
    intakeSub = intake

def stowArm():
    intakeSub.stopIntake()
    armSub.setSetpoint(minArmLength, minArmAngle, 0)