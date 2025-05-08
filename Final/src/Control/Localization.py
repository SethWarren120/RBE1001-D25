from constants import *

# The estimated field position in inches. The origin is the corner next to the ramp
# The X axis is the short side and the Y axis is the long side
fieldX = 74
fieldY = 13

# We store a number of odometry results due to the latency in the vision system
# This allows us to replay the last few odometry updates to account for how far we have moved
# since the frame was captured by the camera to giving the results to the code
wheelEstimates = [[0.0, 0.0],
                  [0.0, 0.0],
                  [0.0, 0.0],
                  [0.0, 0.0],
                  [0.0, 0.0]]

inertial = Inertial(Ports.PORT1)

def odometryEstimate(wheelL, wheelR):
    global fieldX, fieldY
    # Prevent divide by 0
    if wheelL == 0:
        wheelL = 0.0001
    if wheelR == 0:
        wheelR = 0.0001

    # The odometry is calculated radially. That is to say, we assume that the robot is always moving
    # in a circle. This means that we can remove any wheel motion used for rotation from the distance moved
    # Although the arclength is not a perfectly straight line, we assume it is because we are running this over 
    # incredibly small time intervals and is nearly straight
    # The gyro is still the source of the heading, although if there was sufficient time left after completing
    # everything else the rotation value could theoretically be used as an estimate if the gyro disconnected

    # wheelL = left wheel travel distance (arclength), same for wheelR
    # d is the same as above but changed to inner / outer
    # r is the radius for the wheels relative to the turning center
    # no second character means it is for the middle between the wheels

    # Starting from the equations
    # ro = ri + 15
    # d = (ri + 7.5) * theta / (2 * pi)
    # di = ri * theta / (2 * pi)    
    # do = ro * theta / (2 * pi)

    #Solved for unknowns ri, theta, and d
    # ri = 15 * di / (do - di)
    # theta = 2 * pi * di / ri
    # d = (ri + 7.5) * theta / (2 * pi)

    if wheelL < wheelR:
        ri = 15.0 * wheelL / (wheelR - wheelL)
        theta = 2.0 * math.pi * wheelL / ri
        d = (ri + 7.5) * theta / (2 * math.pi)
    elif wheelR < wheelL:
        ri = 15.0 * wheelR / (wheelL - wheelR)
        theta = -(2.0 * math.pi * wheelR / ri)
        d = (ri + 7.5) * -theta / (2 * math.pi)
    else:
        d = wheelL
        theta = 0

    heading = ((360 - inertial.orientation(YAW)) * math.pi / 180.0)
    x = d * math.cos(heading)
    y = d * math.sin(heading)

    wheelEstimates.remove(wheelEstimates[0])
    wheelEstimates.append([x, y])

    fieldX += x
    fieldY += y

# Takes a vision estimate for position from april tags, and then adds the last few odometry estimates to that
def visionEstimate(x, y):
    global fieldX, fieldY
    fieldX = x
    fieldY = y
    for odomEstimate in wheelEstimates:
        fieldX += odomEstimate[0]
        fieldY += odomEstimate[1]