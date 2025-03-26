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
from Subsystems.Drivebase.drivebaseMotorCorrector import *

#Constants
wheelDiameter = 4.0
wheel_travel = math.pi*wheelDiameter
track_width = 11
wheel_base = 11
gear_ratio = 5
wheelCircumference = 3.14 * wheelDiameter 
degreesPerInch = 360.0 / wheelCircumference

brain=Brain()

left_motor = Motor(Ports.PORT10, 18_1, True)
right_motor = Motor(Ports.PORT1, 18_1, False)

left_motor.set_velocity(30, RPM)
left_motor.reset_position()
right_motor.set_velocity(30, RPM)
right_motor.reset_position()

rangeFinderFront = Sonar(Ports.PORT3)
rangeFinderRight = Sonar(Ports.PORT4)

# Drivebase Testing

drivebase = TankDrivebase(left_motor, right_motor, wheelDiameter, gear_ratio, wheel_base, kP=0.5, rangeFinderFront=rangeFinderFront, rangeFinderRightSide=rangeFinderRight)

