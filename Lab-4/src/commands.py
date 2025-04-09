from vex import *
from constants import *
from Subsystems.Drivebase.Tank.tankDrivebase import TankDrivebase

def setSubsystems(driveBase: TankDrivebase):
    global driveSub

    driveSub = driveBase