#!/usr/bin/env python
"""
  Generator 3D maps based on location and laser data
  ... note, run it with FF parameter
""" 

import sys
import math

from localisation import SimpleOdometry, KalmanFilter

class Laser3D:
  def __init__( self, robot, configFilename, verbose = False ):
    self.robot = robot
    self.verbose = verbose
    self.robot.attachLaser()
    self.robot.localisation = SimpleOdometry()
    self.robot.addExtension( self.parseLaserExtension )
#    self.limit = 100

  def parseLaserExtension( self, robot, id, data ):
    if id == 'laser' and len(data) == 541:
      tilt = math.atan2( -0.3, 2.0 ) # TODO tune it
      ct = math.cos( tilt )
      st = math.sin( tilt )
      pose = robot.localisation.pose()
      print "%.3f %.3f 0" % pose[0:2]
      for i in xrange(541):
        a = math.radians((i - 270)/2.0)
        d = data[i]/1000.0
        if data[i] > 0:
          x = d*math.cos(a)                       # tilted
          y = d*math.sin(a)
          heading = pose[2]
          ch = math.cos( heading )
          sh = math.sin( heading )
          print "%.3f %.3f %.3f" % (pose[0] + ch*ct*x - sh*y, pose[1] + ch*y + sh*ct*x, st*x + 0.3)
#      if self.limit > 0:
#        self.limit -= 1
#      else:
#        sys.exit(1)

  def __call__( self ):
    # DO NOTHING ... just parse data available in parseLaserExtension
    while 1:
      self.robot.update()

from eduromaxi import EduroMaxi
import launcher
if __name__ == "__main__": 
  launcher.launch(sys.argv, EduroMaxi, Laser3D)

