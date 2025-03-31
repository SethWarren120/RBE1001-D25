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
from robotConfig import RobotConfig

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

left_motor.set_velocity(200, RPM)
left_motor.reset_position()
right_motor.set_velocity(200, RPM)
right_motor.reset_position()

arm_motor = Motor(Ports.PORT4, 18_1, False)
arm_motor.set_velocity(200, RPM)
arm_motor.reset_position()

rangeFinderFront = Sonar(brain.three_wire_port.e)
rangeFinderRight = Sonar(brain.three_wire_port.g)
lineSensorLeft = Line(brain.three_wire_port.c)
lineSensorRight = Line(brain.three_wire_port.d)
inertial = Inertial(Ports.PORT3)
bumpSwitch = Bumper(brain.three_wire_port.a)

inertial.calibrate()
wait(2000)


# Drivebase Testing
peripherals = [rangeFinderFront, rangeFinderRight, inertial, lineSensorLeft, lineSensorRight, bumpSwitch, arm_motor]
robotConfig = RobotConfig(wheelDiameter, gear_ratio, track_width, peripherals)
drivebase = TankDrivebase(left_motor, right_motor, robotConfig=robotConfig, kP=20)

def printSensors():
    while True:
        #lab 2.4
        brain.screen.print_at("arm current: ", arm_motor.current,x=0,y=20)
        brain.screen.print_at("arm torque: ", arm_motor.torque,x=0,y=40)
        brain.screen.print_at("arm temperature: ", arm_motor.temperature,x=0,y=60)
        brain.screen.render()

sensorsThread = Thread(printSensors)
drivebase.driveLab21()