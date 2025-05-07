from vex import *
from Control.Orchard import *
from constants import tagLocations
from Control.Localization import visionEstimate

cam = AiVision(Ports.PORT2, AiVision.ALL_TAGS)

cam.take_snapshot(AiVision.ALL_TAGS, 8)


def AprilTagLoop():
    while True:
        tags = cam.take_snapshot(AiVision.ALL_TAGS)
        print(len(tags))
        for tag in tags:
            print(tag.id)
            ProcessAprilTag(tag)
        sleep(20)

# aprilTagThread = Thread(AprilTagLoop)

def ProcessAprilTag(aprilTag):
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
        print([fieldx, fieldy])
        visionEstimate(fieldx, fieldy)

