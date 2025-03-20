# ---------------------------------------------------------------------------- #
#                                                                              #
# 	Module:       main.py                                                      #
# 	Author:       ZBurgoon                                                     #
# 	Created:      3/20/2025, 11:51:15 AM                                       #
# 	Description:  V5 project                                                   #
#                                                                              #
# ---------------------------------------------------------------------------- #

# Library imports
from vex import *

# Brain should be defined by default
brain=Brain()

brain.screen.print("Hello V5")

gearRatio = 5
# Brain should be defined by default
left_motor = Motor(Ports.PORT1, 18_1, True)
right_motor = Motor(Ports.PORT10, 18_1, False)
arm_motor = Motor(Ports.PORT8, 18_1, True)

brain.screen.print("Hello V5")

left_motor.spin_for(FORWARD, 1 * gearRatio, TURNS, 30, RPM, False)
right_motor.spin_for(FORWARD, 1 * gearRatio, TURNS, 30, RPM, True)

wait(3000)
# arm_motor.spin_for(REVERSE, .5, TURNS, 100, RPM)

        
