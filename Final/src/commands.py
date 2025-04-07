from vex import *
from constants import *

def getSubsystems(driveBase, arm, intake, forks):
    global driveSub, armSub, intakeSub, forksSub

    driveSub = driveBase
    armSub = arm
    intakeSub = intake
    forksSub = forks

def grabFruit(fruitPose, height):
    armSub.toPosition(height)
    driveSub.driveToPose(fruitPose[0], fruitPose[1], 0)