from vex import *
from constants import *

def setSubsystems(driveBase, arm, intake, forks):
    global driveSub, armSub, intakeSub, forksSub

    driveSub = driveBase
    armSub = arm
    intakeSub = intake
    forksSub = forks

def grabFruit(fruitPose, height):
    armSub.toPosition(height, lowFruitAngle)
    intakeSub.intakeUntilCurrent()
    
    armLength = armSub.getHeight()  # Get the current arm length
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
    armSub.toPosition(minArmLength, minArmAngle)
    intakeSub.stopIntake()