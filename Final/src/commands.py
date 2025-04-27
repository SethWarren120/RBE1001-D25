from vex import *
from constants import *
from Subsystems.Drivebase.Holonomic.mecanumDrivebase import MecanumDrivebase
from Subsystems.Arm.arm import Arm
from Subsystems.Intake.intake import Intake
from Subsystems.Forks.forks import Forks

def setSubsystems(driveBase: MecanumDrivebase, arm: Arm, intake: Intake, forks: Forks):
    global driveSub, armSub, intakeSub, forksSub

    driveSub = driveBase
    armSub = arm
    intakeSub = intake
    forksSub = forks

def grabFruit(fruitPose, length, isLowFruit):
    if isLowFruit:
        armSub.setSetpoint(length, lowFruitAngle, lowFruitWristAngle)
    else:
        armSub.setSetpoint(length, highFruitAngle, highFruitWristAngle)
    intakeSub.intakeUntilCurrent()
    
    armLength = armSub.getLength()  # Get the current arm length
    armAngle = math.radians(armSub.getAngle())  # Convert arm angle to radians
    
    robotHeading = math.radians(driveSub.heading) # uses the heading in radians

    offsetX = armLength * math.cos(robotHeading) # Get the x offset of the arm relative to the robot
    offsetY = armLength * math.sin(robotHeading) # Get the y offset of the arm relative to the robot

    adjustedX = fruitPose[0] - offsetX
    adjustedY = fruitPose[1] - offsetY

    deltaX = fruitPose[0] - driveSub.x  # get the change in x position
    deltaY = fruitPose[1] - driveSub.y  # get the change in y position
    targetHeading = math.degrees(math.atan2(deltaY, deltaX))  # find the angle the robot neeeds to turn to face the fruit
    
    driveSub.driveToPose(adjustedX, adjustedY, targetHeading)

def stowArm():
    intakeSub.stopIntake()
    armSub.setSetpoint(minArmLength, minArmAngle, minWristAngle)