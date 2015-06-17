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
from localisation import SimpleOdometry, KalmanFilter

cfgDrawPosition = False
WEED_LIMIT = 1400

UNKNOWN = (0xbe, 0xbe, 0xbe) # Unknown/unexplored area: GRAY
PLANS = 0x0       # Plants/Obstacles/Walls: BLACK
FREE = 0xffffff   # Free known area/explored area: WHITE
MARKED = 0xff0000 # Marked plants/detection target: RED

scale = 100.0  # i.e. 1 pixel = 1cm
imsize = (1500,2000)
offsetX, offsetY = imsize[0]/2, imsize[1]/2+300

def replace( arr, val, newVal ):
    ret = []
    for a in arr:
        if a == val:
            ret.append( newVal )
        else:
            ret.append( val )
    return ret


def filterLocalMinima( laserData, neiSize=15 ):
    "replace all non-zero elements by zero if they are not local minima"
    laserData = laserData[:] # own copy
    replace(laserData,0,10000)
    ret = [0]*neiSize # laserData[:neiSize]
    for i in xrange(neiSize, len(laserData)-neiSize):        
        if min(laserData[i-neiSize:i+neiSize+1]) == laserData[i]:
            ret.append( laserData[i] )
        else:
            ret.append( 0 )
    ret.extend( [0]*neiSize ) #laserData[-neiSize:] )
    return ret


def createMap( robot ):
    img = np.zeros( (imsize[1],imsize[0],3), np.uint8 )
    cv2.rectangle( img, (0,0), (img.shape[1], img.shape[0]), UNKNOWN, -1 )
    prevLaser = None
    prevCamera = None
    weeds = []
    try:
        for tick in xrange(5000):
            robot.update()
            (x,y,heading) = robot.localisation.pose()
            if cfgDrawPosition:
                cv2.circle( img, (offsetX + int(x*scale), offsetY -int(y*scale)), 2, (255,0,0) )
            if robot.laserData and robot.laserData != prevLaser:
                endOfRow = True
                for i,d in enumerate(filterLocalMinima(robot.laserData)):
                    angle = heading + math.radians((i-270)/2.)
                    dist = d/1000.0
                    if d > 0 and dist < 0.75:
                        sx = x + dist * math.cos( angle )
                        sy = y + dist * math.sin( angle )
                        cv2.circle( img, (offsetX + int(sx*scale), offsetY -int(sy*scale)), 1, (0,0,0) )
                        cv2.line( img, (offsetX + int(sx*scale), offsetY -int(sy*scale)), 
                                 (offsetX + int(x*scale), offsetY-int(y*scale)), (255,255,255) )
                        endOfRow = False
                if endOfRow:
                    if cfgDrawPosition:
                        cv2.circle( img, (offsetX + int(x*scale), offsetY-int(y*scale)), 2, (0,0,255) )
                prevLaser = robot.laserData
            if robot.cameraData and robot.cameraData != prevCamera:
                if robot.cameraData[0]:
                    dist = -0.26
                    sx = x + dist * math.cos( heading )
                    sy = y + dist * math.sin( heading )
                    left, right = [int(v) for v in robot.cameraData[0].split()[:2]]
                    if left > WEED_LIMIT:
                        weeds.append( (offsetX + int((sx-0.35*math.sin(heading))*scale), offsetY-int((sy+0.35*math.cos(heading))*scale)) )
                    if right > WEED_LIMIT:
                        weeds.append( (offsetX + int((sx+0.35*math.sin(heading))*scale), offsetY-int((sy-0.35*math.cos(heading))*scale)) )
                prevCamera = robot.cameraData
    except LogEnd:
        pass
    for w in weeds:
        cv2.circle( img, w, 10, (0,0,255), -1 )
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
    robot.localisation = KalmanFilter() #SimpleOdometry()
    robot.attachLaser( remission=True )
    robot.attachCamera()
    return robot

if __name__ == "__main__":
    if len(sys.argv) < 3:
        print __doc__
        sys.exit(2)

    img = createMap( createRobot(sys.argv[1]) )
    cv2.imwrite( sys.argv[2], img )

# vim: expandtab sw=4 ts=4 


