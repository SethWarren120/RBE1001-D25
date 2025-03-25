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

#Constants
wheelDiameter = 4.0
wheel_travel = math.pi*wheelDiameter
track_width = 11
wheel_base = 11
gear_ratio = 5
wheelCircumference = 3.14 * wheelDiameter 
degreesPerInch = 360.0 / wheelCircumference
kP = 0.5

brain=Brain()

left_motor = Motor(Ports.PORT10, GearSetting.RATIO_18_1, False)
left_motor.set_velocity(30, RPM)
left_motor.reset_position()
right_motor = Motor(Ports.PORT1, GearSetting.RATIO_18_1, False)
right_motor.set_velocity(30, RPM)
right_motor.reset_position()

rangeFinderFront = Sonar(Ports.PORT4)
rangeFinderRightSide = Sonar(Ports.PORT3)
        
def updateOdometry():
    while True:
        left_motor.reset_position()
        right_motor.reset_position()

        wait(1500, MSEC)

        left_position = left_motor.position(DEGREES)
        right_position = right_motor.position(DEGREES)
        brain.screen.print_at("left: " + str(left_position), x=0, y=80)
        brain.screen.print_at("right: " + str(right_position), x=0, y=100)

        left_distance = (left_position / 360.0) * wheelCircumference / gear_ratio
        right_distance = (right_position / 360.0) * wheelCircumference / gear_ratio

        avg_distance = (left_distance + right_distance) / 2.0
        delta_theta = (right_distance - left_distance) / track_width
        
        brain.screen.print_at("avg_distance: " + str(avg_distance), x=0, y=120)
        brain.screen.print_at("delta_theta: " + str(delta_theta), x=0, y=140)

        odom[0] += avg_distance * math.cos(math.radians(odom[2]))
        odom[1] += avg_distance * math.sin(math.radians(odom[2]))
        odom[2] += math.degrees(delta_theta)
        if (odom[2] > 360):
            odom[2] -= 360
        elif (odom[2] < 0):
            odom[2] += 360

        brain.screen.print_at("X: " + str(odom[0]) + "       pX: " + str(pOdom[0]), x=0, y=0)
        brain.screen.print_at("Y: " + str(odom[1]) + "       pY: " + str(pOdom[1]), x=0, y=20)
        brain.screen.print_at("Heading: " + str(odom[2]) + "       pHeading: " + str(pOdom[2]), x=0, y=40)
        brain.screen.render()

#x y theta
#when theta is 0: +x is forward, +y is left
odom = [0.0,0.0,0.0]
pOdom = [0.0,0.0,0.0]
odomThread = Thread(updateOdometry)

def moveLen(len):
    deg = 360 * ((len / wheelCircumference) * gear_ratio)

    left_motor.spin_for(FORWARD, deg, DEGREES, 100, RPM, False)
    right_motor.spin_for(FORWARD, deg, DEGREES, 100, RPM, True)

    pOdom[0] += len * math.cos(math.radians(odom[2]))
    pOdom[1] += len * math.sin(math.radians(odom[2]))

def drive(speed, direction):
    motor_direction = FORWARD if speed >= 0 else REVERSE
    speed_abs = abs(speed)
    direction_normalized = max(min(direction / 100.0, 1.0), -1.0)
    left_speed = speed_abs
    right_speed = speed_abs
    
    if direction_normalized > 0:  # Turn left
        left_speed = speed_abs * (1.0 - direction_normalized)
    elif direction_normalized < 0:  # Turn right
        right_speed = speed_abs * (1.0 + direction_normalized)

    left_motor.spin(motor_direction, left_speed, RPM, False)
    right_motor.spin(motor_direction, right_speed, RPM, True)

def wallFollowInches(setDistanceFromWall):
    actualSetpoint = setDistanceFromWall - 1.0
    minDistance = 1.0  # Minimum safe distance (inches)
    maxDistance = 20.0  # Maximum reliable sensor distance (inches)
    
    try:
        while True:
            # Get current distance from wall
            currentDistance = rangeFinderRightSide.distance(INCHES)
            
            error = currentDistance - actualSetpoint
            
            if currentDistance < minDistance:
                steeringCorrection = 75  # Turn left (away from wall)
            elif currentDistance > maxDistance:
                steeringCorrection = -50  # Turn right (toward wall)
            else:
                steeringCorrection = kP * error
                steeringCorrection = max(min(steeringCorrection, 100), -100)
            
            drive(100, steeringCorrection)

            #If it is <8 inches from the front wall, turn left
            if (rangeFinderFront.distance(INCHES) < 8):
                drive(30,100)
    finally:
        # Stop motors when done
        left_motor.stop()
        right_motor.stop()

def turnDeg(deg, pivotOffset = 0.0):
    lenLeft = -(2 * 3.14 * ((track_width / 2) + pivotOffset)) * (-deg / 360)
    lenRight = (2 * 3.14 * ((track_width / 2) - pivotOffset)) * (-deg / 360)

    degLeft = 360 * ((lenLeft / wheelCircumference) * gear_ratio)
    degRight = 360 * ((lenRight / wheelCircumference) * gear_ratio)

    if (degLeft > degRight):
        left_motor.spin_for(FORWARD, degLeft, DEGREES, 100, RPM, False)
        right_motor.spin_for(FORWARD, degRight, DEGREES, 100 * (degRight / degLeft), RPM, True)
    else:
        left_motor.spin_for(FORWARD, degLeft, DEGREES, 100 * (degLeft / degRight), RPM, False)
        right_motor.spin_for(FORWARD, degRight, DEGREES, 100, RPM, True)

    theta_rad = math.radians(odom[2])
    deg_rad = math.radians(deg)
    radius = abs(pivotOffset)
    if deg != 0:  # Avoid calculations if no rotation
        # Calculate the change in position (works for all pivotOffset values)
        dx = radius * (math.cos(theta_rad + deg_rad) - math.cos(theta_rad))
        dy = radius * (math.sin(theta_rad + deg_rad) - math.sin(theta_rad))
        
        # Update predicted position
        pOdom[0] += dx
        pOdom[1] += dy
    
    pOdom[2] += deg
    if pOdom[2] > 360:
        pOdom[2] -= 360
    elif pOdom[2] < 0:
        pOdom[2] += 360

def driveToPose(x,y,theta):
    currentPose = odom
    # deltaX = x - currentPose[0]
    # deltaY = y - currentPose[1]
    # deltaTheta = theta - currentPose[2]

    # distance = math.sqrt(deltaX**2 + deltaY**2)
    # angle = math.degrees(math.atan2(deltaY, deltaX))
    
    # turnDegrees(angle - currentPose[2], 5)
    # driveDistance(distance, 5)
    # turnDegrees(theta - angle, 5)

wallFollowInches(11)