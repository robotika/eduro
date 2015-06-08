#!/usr/bin/env python
"""
  Field Robot Event 2015 - create map of the maize field
  usage:
       ./frelog2map.py <metalog> <output image file>
"""

#from fre import *
import sys
import cv2
import math
import numpy as np

from eduromaxi import EduroMaxi
from can import CAN, ReplyLogInputsOnly, LogEnd
from localisation import SimpleOdometry

UNKNOWN = (0xbe, 0xbe, 0xbe) # Unknown/unexplored area: GRAY
PLANS = 0x0       # Plants/Obstacles/Walls: BLACK
FREE = 0xffffff   # Free known area/explored area: WHITE
MARKED = 0xff0000 # Marked plants/detection target: RED

scale = 100.0  # i.e. 1 pixel = 1cm
imsize = (2000,1500)


def createMap( robot ):
    img = np.zeros( (imsize[1],imsize[0],3), np.uint8 )
    cv2.rectangle( img, (0,0), (img.shape[1], img.shape[0]), UNKNOWN, -1 )
    prevLaser = None
    try:
        for tick in xrange(500):
            robot.update()
            (x,y,heading) = robot.localisation.pose()
            cv2.circle( img, (int(x*scale)+imsize[0]/2, -int(y*scale)+imsize[1]/2), 2, (255,0,0) )
            if robot.laserData and robot.laserData != prevLaser:
                for i,d in enumerate(robot.laserData):
                    angle = heading + math.radians((i-270)/2.)
                    dist = d/1000.0
                    if d > 0 and dist < 0.5:
                        sx = x + dist * math.cos( angle )
                        sy = y + dist * math.sin( angle )
                        cv2.circle( img, (int(sx*scale)+imsize[0]/2, -int(sy*scale)+imsize[1]/2), 1, (0,0,0) )
                prevLaser = robot.laserData
    except LogEnd:
        pass
    return img
    

def createRobot( logName, runNumber=0 ):
    metaLog = open(logName)
    for line in metaLog:
        if line.startswith("CANlog:"):
            runNumber -= 1
            if runNumber <= 0:
                logName = logName[:logName.find("meta")]+line.split()[1].split("/")[-1]
                break    
    can = CAN( ReplyLogInputsOnly( logName ), skipInit = True )
    robot = EduroMaxi( can, replyLog=logName, metaLog=metaLog)
    robot.localisation = SimpleOdometry()
    robot.attachLaser( remission=True )
    return robot

if __name__ == "__main__":
    if len(sys.argv) < 3:
        print __doc__
        sys.exit(2)

    img = createMap( createRobot(sys.argv[1]) )
    cv2.imwrite( sys.argv[2], img )

# vim: expandtab sw=4 ts=4 


