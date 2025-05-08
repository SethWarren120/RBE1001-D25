from vex import *
from Control.Orchard import *
from constants import tagLocations
from Control.Localization import visionEstimate

cam = AiVision(Ports.PORT2, AiVision.ALL_TAGS)

cam.take_snapshot(AiVision.ALL_TAGS, 8)

# April tag async thread function
def AprilTagLoop():
    while True:
        tags = cam.take_snapshot(AiVision.ALL_TAGS)
        print(len(tags))
        for tag in tags:
            print(tag.id)
            ProcessAprilTag(tag)
        sleep(20)

# By plotting experimental values for size in pixels vs distance, we were able to get a function for each size april tag
# That would tell us how far away it was for its size. We required different functions for each because the tree april tags were
# much smaller

def ProcessAprilTag(aprilTag):
    # If its id is 0 - 11, then it is a tag on a tree branch
    if aprilTag.id < 12:
        distance = 290 / aprilTag.height
        angle = 27.5 - (27.5 * (aprilTag.centerY)/120)
        height = math.sin(angle * 2 * math.pi / 360) * distance
        GiveTreeHeightEstimate(aprilTag.id, height, aprilTag.height)
        cr = 0 #Current rotation, get from gyro
        GiveTreeRotationEstimate(aprilTag.id, cr + aprilTag.rotation, aprilTag.height)
    else:
        distance = 730 / aprilTag.height
        fieldAngle = math.pi * (360 - (aprilTag.angle - tagLocations[aprilTag.id - 12][2])) / 180        
        fieldx = tagLocations[aprilTag.id - 12][0] + distance * math.cos(fieldAngle)
        fieldy = tagLocations[aprilTag.id - 12][1] + distance * math.sin(fieldAngle)
        visionEstimate(fieldx, fieldy)

