# Library imports
from vex import *

def maxRotationUnits(rotationUnits):
    match rotationUnits:
        case RotationUnits.DEG:
            return 360.0
        case RotationUnits.REV:
            return 1.0
                
        case _:
            return 360
