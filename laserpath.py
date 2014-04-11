#!/usr/bin/python
"""
  Laser Path - scan obstalce from laser?? (origin Robotour/FRE)
"""

import sys
import math

from ray_trace import combinedPose
from line import Line
from itertools import islice

class LaserPath:
  FAR_AWAY = 100000

  def __init__( self, verbose = False, sensorID="laser" ):
    self.verbose = verbose
    self.sensorID = sensorID
    self.reset()

  def reset( self, pose = (0,0,0) ):
    print "RESET ROW (%0.2f, %0.2f, %0.1f)" % (pose[0], pose[1], math.degrees(pose[2]) )
    self.preference = None
    self.directionAngle = 0.0
    self.offset = 0
    goal = combinedPose( (pose[0], pose[1], pose[2]+self.directionAngle), (1.0, 0, 0) ) # was radius
    self.line = Line( pose, goal )
    self.newData = False
    self.traversable = None
    self.obstacles = []
    self.step = 10 # how many neighboring measurements are merged together
    #TODO: get the laser parameters somewhere
    h = 310 # Height of the laser above ground. [mm]
    alpha = math.radians(81.0) # An angle between a vertical direction and a forward measurement beam.
    n = 541 # Number of values provided by the laser
    self.laserRight = math.radians(-135.0) # Left-most angle measured by the laser. [rad]
    self.laserLeft = math.radians(+135.0) # Right-most angle measured by the laser. [rad]
    self.expectation = [] # Expected distances to the ground plane. [mm]
    epsilon = 1.0e-3
    angle_threshold = math.pi / 3 #math.pi / 2.0 - epsilon
    for i in xrange(n):
      beta = self.laserRight + (self.laserLeft - self.laserRight) * i / float(n)
      if abs(beta) >= angle_threshold:
        beta = angle_threshold * (1 if beta > 0 else -1) # Making sure we always have an "intersection" in a reasonable
      x = h / (math.cos(alpha) * math.cos(beta))
      self.expectation.append(x)
    avg_decim = lambda sl: sum(sl) / self.step
    self.decimatedExpectation = self.__decimate(self.expectation, decimator = avg_decim)

  def updateExtension( self, robot, id, data ):
    if id == self.sensorID and len(data) > 0:
      # Is there an obstacle somewhere very close?
      lbound = len(data) / 6
      rbound = len(data) * 5 / 6
      # (lbound, rbound) should cover the space ahead of the robot.
      obstacle_threshold = 300 # [mm]
      for x in islice(data, lbound, rbound):
        if x > 0 and x < obstacle_threshold:
            self.line = None
            self.newData = True
            self.directionAngle = 0.0
            if self.verbose:
              s = ''
              for start in xrange(0, len(data), self.step):
                too_close = False
                for x in islice(data, start, start+self.step):
                  if x > 0 and x < obstacle_threshold:
                    too_close = True
                    break
                s += '!' if too_close else '-'
              print "'" + s[::-1] + "'" # The reverse is needed to match the printed traversability pattern.
            self.traversable = None
            self.obstacles = []
            return

      # Preprocess the data, so that the computation is not repeated multiple times.
      prepData = [ self.FAR_AWAY if x == 0 else x for x in data ]
      backData = [ self.expectation[i] - x for (i, x) in enumerate(prepData) ] # distances measured from the expected ground distance

      # Traversability stemming from "there is no obstacle close there".
      decimatedPrepData = self.__decimate(prepData, decimator = min)
      decimatedBackData = self.__decimate(backData, decimator = max)
      trav_obs = self.obstacleTraversability( decimatedBackData )

      self.traversable = trav_obs

      self.obstacles = []
      for i in xrange(len(self.traversable)):
        if not self.traversable[i]:
          phi = self.laserRight + (self.laserLeft - self.laserRight) * i / (len(self.traversable) - 1)
          dist = decimatedPrepData[i] / 1000.0
          self.obstacles.append( (phi, dist) )

      center = len(self.traversable)/2 + self.offset
      if center < 0: center = 0
      elif center >= len(self.traversable): center = len(self.traversable) - 1

      # Look for a traversable direction. The more in the expected direction, the better.
      for i in range(max(center, len(self.traversable) - center)):
        if center - i >= 0 and self.traversable[center - i]:
          center -= i
          break
        elif center + i < len(self.traversable) and self.traversable[center + i]:
          center += i
          break

      #If the center is not traversable, we should better STOP!!!
      if not self.traversable[center]:
        self.line = None
        self.newData = True
        self.directionAngle = 0.0
        if self.verbose:
          print '+' * len(self.traversable)
        self.traversable = None
        self.obstacles = []
        return

      # Find borders of the traversable gap.
      i = 0
      while i < center:
        if not self.traversable[center - i]:
          break
        i += 1
      right = i
      i = 0
      while i + center < len(self.traversable):
        if not self.traversable[center + i]:
          break
        i += 1
      left = i

      # Spit the information.
      s = ''
      for t in self.traversable:
        s += (' ' if t else 'x')
      s = "".join( list(s)[:center] + ['C'] + list(s)[center:] )
      s = s[::-1]
      if self.verbose:
        print "'" + s + "'"      
#      print center, left, right, left+right

      # Update the belief.
      if left+right <= 10: # TODO limit based on minSize, radius and sample angle
        self.offset += (left - right) / 2
        self.directionAngle = math.radians( self.offset * self.step / 2.0 )
      elif left < 5 or right < 5:
        # wide row & collision ahead
        print left, right, left+right
        if right < left:
          self.offset += 5
        else:
          self.offset -= 5
        self.directionAngle = math.radians( self.offset * self.step / 2.0 )
      else:
#        print "OFF", left, right, left+right
        self.directionAngle = 0.0 # if you do not know, go ahead

      pose = robot.localisation.pose()
      goal = combinedPose( (pose[0], pose[1], pose[2]+self.directionAngle), (1.0, 0, 0) )
      self.line = Line( pose, goal )
      self.newData = True
    if id == 'camera':
      if self.verbose:
        print data[1]

  def __decimateG(self, data, decimator):
    ''' Decimate the given data chunk-by-chunk (self.step size) using the given decimator as an aggregation function. '''
    return ( decimator(sl) for sl in (islice(data, start, start + self.step) for start in xrange(0, len(data), self.step)) )

  def __decimate(self, data, decimator):
    ''' Decimate the given data chunk-by-chunk (self.step size) using the given decimator as an aggregation function. '''
    return list(self.__decimateG(data, decimator))

  def roughnessTraversability(self, prepData, radius = 5):
    r = roughness(prepData)
    limit = (2 * radius + 1) * (45 ** 2) # roughness limit # Note: #window_elements * dist_diff**2
    return self.__decimate(r, decimator = lambda sl: all(x <= limit for x in sl))

  def obstacleTraversability(self, data):
    NEARNESS = 0.4 # Nearness factor. [% / 100]. If an obstacle is closer than this fraction of the expected distance, it is a no-go.
    traversable = [ m < (1.0 - NEARNESS) * self.decimatedExpectation[i] for (i, m) in enumerate(data) ]
    return traversable

  def isTraversable(self, angle):
    if self.traversable is None or not self.traversable:
      return False # either no data, or a very close obstacle

    bin = int( (angle - self.laserRight) / (self.laserLeft - self.laserRight) * len(self.traversable) )

    # some rounding errors are possible, thus:
    if bin < 0.0: bin = 0.0
    if bin >= len(self.traversable): bin = len(self.traversable) - 1

    return self.traversable[bin]

if __name__ == "__main__": 
  if len(sys.argv) < 2:
    print __doc__
    sys.exit(-1)
  laserPath = LaserPath( verbose=True )
#  robot = None
#  laserPath.updateExtension( robot, "laser", [200]*541 )

