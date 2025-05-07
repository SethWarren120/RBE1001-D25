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

tracking_enabled = False

def startObjectTracking():
    global tracking_enabled
    tracking_enabled = True
    tracking_thread = Thread(continuousArmTracking)

def stopObjectTracking():
    global tracking_enabled
    tracking_enabled = False

def continuousArmTracking():
    global tracking_enabled
    
    while tracking_enabled:
        findAndAimAtObject()
        wait(50)

def findAndAimAtObject():
    object = None
    
    objectsGreen = driveSub.camera.take_snapshot(vision_Green)
    objectsOrange = driveSub.camera.take_snapshot(vision_Orange)
    objectsYellow = driveSub.camera.take_snapshot(vision_Yellow)

    largestwidth = 0
    for tobject in objectsGreen:
        if tobject.width > largestwidth:
            largestwidth = tobject.width
            object = tobject
    
    for tobject in objectsOrange:
        if tobject.width > largestwidth:
            largestwidth = tobject.width
            object = tobject
    
    for tobject in objectsYellow:
        if tobject.width > largestwidth:
            largestwidth = tobject.width
            object = tobject

    if object is None:
        return
    
    y = object.centerY
    correctedY = cameraYOffset - y
    
    angleAdjustment = correctedY * (pivotPID[0] / (cameraHeight / 2))
    currentAngle = armSub.getAngle()
    targetAngle = currentAngle + angleAdjustment
    
    armSub.setSetpoint(
        armSub.getLength(),
        targetAngle,
        armSub.getWristAngle()
    )

def angleArmToObject():
    startObjectTracking()

def dumpObject():
    currentWristAngle = armSub.getWristAngle()
    if currentWristAngle < 180 and currentWristAngle > 0:
        armSub.setSetpoint(140, 20, 90)
    elif currentWristAngle > 180:
        armSub.setSetpoint(140, 20, 270)
    else:
        armSub.setSetpoint(140, 20, -90)
    
    sleep(3000)
    intakeSub.runIntakeForTime(REVERSE, 2000)
    sleep(3000)
    armSub.stowArm()