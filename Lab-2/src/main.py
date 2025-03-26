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

left_motor.set_velocity(200, RPM)
left_motor.reset_position()
right_motor.set_velocity(200, RPM)
right_motor.reset_position()

rangeFinderFront = Sonar(brain.three_wire_port.e)
rangeFinderRight = Sonar(brain.three_wire_port.g)
lineSensorLeft = Line(brain.three_wire_port.c)
lineSensorRight = Line(brain.three_wire_port.d)
inertial = Inertial(Ports.PORT3)
inertial.calibrate()

# Drivebase Testing

sensorList = [rangeFinderFront, rangeFinderRight, inertial, lineSensorLeft, lineSensorRight]
drivebase = TankDrivebase(left_motor, right_motor, sensorList, wheelDiameter, gear_ratio, wheel_base, kP=0.5)

def printSensors():
    while True:
        brain.screen.print_at("Front Range Finder: ", rangeFinderFront.distance(INCHES),x=0,y=20)
        brain.screen.print_at("Right Range Finder: ", rangeFinderRight.distance(INCHES),x=0,y=40)
        brain.screen.print_at("Left Line Sensor: ", lineSensorLeft.reflectivity(),x=0,y=60)
        brain.screen.print_at("Right Line Sensor: ", lineSensorRight.reflectivity(),x=0,y=80)
        brain.screen.print_at("Inertial: ", inertial.heading(),x=0,y=100)
        brain.screen.print_at("direction: ", 0.5*(rangeFinderRight.distance(INCHES)-8),x=0,y=120)
        brain.screen.print_at("speed1: ", 100+0.5*(rangeFinderRight.distance(INCHES)-8),x=0,y=140)
        brain.screen.print_at("speed2: ", 100-0.5*(rangeFinderRight.distance(INCHES)-8),x=0,y=160)
        brain.screen.render()


sensorsThread = Thread(printSensors)
drivebase.driveLab21()