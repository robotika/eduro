#!/usr/bin/python
"""
   Follow Me, so that robot moves on the right side of the person
"""

import sys
import math
import itertools
from itertools import izip

from can import CAN, ReplyLog, ReplyLogInputsOnly
from localisation import SimpleOdometry
from driver import Driver, normalizeAnglePIPI, angleTo
from robot import SourceLogger

from eduro import EmergencyStopException, emergencyStopExtension
from eduromaxi import EduroMaxi, buildRobot
import starter

from simulator import Simulator

from route import Convertor
from line import distance, Line
from ray_trace import combinedPose

import viewlog
from viewlog import viewLogExtension, viewCompassExtension, viewPoseExtension
from camera import img2xy

from laseranalysis import ObstacleAvoidance, LaserViewer

def ver0( robot, verbose = False ):
  masterAngleOffset = math.radians(0) # on FRE2012 was used 45
  robot.fnSpeedLimitController = [robot.pauseSpeedFn] 

#  obstacleAvoidance = ObstacleAvoidance() # or robot, or driver??
#  robot.addExtension( obstacleAvoidance.updateExtension )
#  robot.fnSpeedLimitController.append( obstacleAvoidance.speedController ) 

  robot.attachEmergencyStopButton() 
  robot.attachSonar() 
  robot.laser.stopOnExit = False
#  robot.gps.start()
  robot.laser.start()
  robot.camera.start()
  robot.localisation = SimpleOdometry() 

  thresholds = []
  for i in xrange(541):
    deg = -135 + i * 0.5
    rad = math.radians(deg)
    thresh = 1000 * (0.17 + 0.17 * max(0, math.cos(rad))) # [mm]
    thresholds.append(thresh)

  try:
    index = None
    while True:
      if robot.laserData:
        near = 10.0
        if index is None:
          low = 0
          high = 541
        else:
          low = max(0, index - 20)
          high = min(541, index + 20)
        for i in range(low,high):
          x = robot.laserData[i]
          if x != 0 and x < near*1000:
            near = x/1000.0
            index = i
        maxnear = min( (x for x in robot.laserData if x > 0) ) / 1000.0
        if verbose:
          print near, maxnear,index
        if near > 1.3 or any(x < thresh for (x, thresh) in izip(robot.laserData, thresholds) if x > 0):
          robot.setSpeedPxPa( 0.0, 0.0 )
        else:
          angle = math.radians( (index-270)/2.0 ) + masterAngleOffset
          desiredAngle = 0
          desiredDistance = 0.4
          speed = 0.2 + 2*(near - desiredDistance)
          rot = 1.5 * (angle - desiredAngle)
          if speed < 0:
            speed = 0
          robot.setSpeedPxPa( speed, rot )
      robot.update()
  except EmergencyStopException, e:
    print "EmergencyStopException"
  robot.camera.requestStop()
#  robot.gps.requestStop()
  robot.laser.requestStop()



def ver1( robot, verbose = False ):
  robot.fnSpeedLimitController = [robot.pauseSpeedFn] 

  obstacleAvoidance = ObstacleAvoidance() # or robot, or driver??
  robot.addExtension( obstacleAvoidance.updateExtension )
  robot.fnSpeedLimitController.append( obstacleAvoidance.speedController ) 

  robot.addExtension( emergencyStopExtension ) 
  robot.laser.stopOnExit = False
  robot.gps.start()
  robot.laser.start()
  robot.camera.start()
 
  try:
    while True:
      if robot.laserData:
        near = 10.0
#        for i in range(360,541): # TODO verify that it is left
        for i in range(300,541): # TODO verify that it is left
          x = robot.laserData[i]
          if x != 0 and x < near*1000:
            near = x/1000.0
            index = i
        if verbose:
          print near, index
        if near > 1.3:
          robot.setSpeedPxPa( 0.0, 0.0 )
        else:
          # TODO something based on near and index
          angle = math.radians( (index-270)/2.0 )
          desiredAngle = math.radians(75)
#          desiredDistance = 0.5
          desiredDistance = 0.4
          turnOffset = math.radians( 30*(near - desiredDistance) )
          speed = math.sin(desiredAngle-angle)
          robot.setSpeedPxPa( speed, (angle - desiredAngle)+turnOffset )
      robot.update()
  except EmergencyStopException, e:
    print "EmergencyStopException"
  robot.camera.requestStop()
  robot.gps.requestStop()
  robot.laser.requestStop()

class FollowMe:
  def __init__(self, robot, configFile=None, verbose = False):
    self.robot = robot
    self.verbose = verbose

    MAX_ACC = 1.4
    self.robot.maxAcc = MAX_ACC
#    self.robot.attachGPS()
    self.robot.attachLaser()
#    self.robot.attachCamera()
    self.robot.attachCamera( cameraExe = "../robotchallenge/rc" )
    self.robot.addExtension( emergencyStopExtension )

  def __call__(self):
    ver0(self.robot, self.verbose)


from eduromaxi import EduroMaxi
import launcher
if __name__ == "__main__": 
  if len(sys.argv) < 2:
    print __doc__
    sys.exit(-1)
  launcher.launch(sys.argv, EduroMaxi, FollowMe)

