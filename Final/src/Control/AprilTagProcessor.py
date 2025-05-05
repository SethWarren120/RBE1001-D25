from vex import *

cam = AiVision(Ports.PORT1, AiVision.ALL_TAGS)

cam.take_snapshot(AiVision.ALL_TAGS, 8)

def ProcessAprilTag(aprilTag):
    if aprilTag.id < 12:
        pass #Branch Detection
    else:
        pass #Robot Positioning