from constants import *

fieldX = 74
fieldY = 13

wheelEstimates = [[0.0, 0.0],
                  [0.0, 0.0],
                  [0.0, 0.0],
                  [0.0, 0.0],
                  [0.0, 0.0]]

inertial = Inertial(Ports.PORT1)

def odometryEstimate(wheelL, wheelR):
    global fieldX, fieldY
    if wheelL == 0:
        wheelL = 0.0001
    if wheelR == 0:
        wheelR = 0.0001
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

def visionEstimate(x, y):
    global fieldX, fieldY
    fieldX = x
    fieldY = y
    for odomEstimate in wheelEstimates:
        fieldX += odomEstimate[0]
        fieldY += odomEstimate[1]