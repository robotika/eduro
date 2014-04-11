#!/usr/bin/python
"""
  Tool for analysis of laser data
  usage:
      laseranalysis.py <log file>
"""

import sys
import math
from robot import SourceLogger 
import viewlog
from itertools import islice

from line import distance, Line 
from ray_trace import combinedPose 

class LaserViewer:
  def __init__( self, robot, degStep = 5, distanceLimit = 1.5, sideLimit = None ):
    self.degStep = degStep
    self.distanceLimit = distanceLimit
    self.sideLimit = sideLimit
    robot.addExtension( self.viewLaserExtension )
    self.firstData = True

  def dumpLaserGeometry(self, pose=None ):
    if pose == None:
      geo = [(0,0,math.radians(angle)) for angle in range(-135, 136, self.degStep)]
    else:
      (x,y,z),(a,b,c) = pose
      geo = [(x,y,a+math.radians(angle)) for angle in range(-135, 136, self.degStep)]
      assert( b == 0 or b == math.radians(180) )
      if b == math.radians(180):
        geo.reverse()
    viewlog.dumpSharpsGeometry( geo )

  def viewLaserExtension( self, robot, id, data ):
    if id == 'laser' or id == 'laser2':
#      if self.firstData:
#        self.dumpLaserGeometry( robot.laser.pose )
#        self.firstData = False
      if id == 'laser':
        self.dumpLaserGeometry( robot.laser.pose )
      else:
        self.dumpLaserGeometry( robot.laser2.pose )

      if robot.localisation:
        s = 2*self.degStep
        data2 = [ (x == 0 or x > self.distanceLimit*1000) and 100000 or x for x in data]
        if self.sideLimit:
          if len(data) == 541:
            for i in range(90-20, 90+20) + range(540-90-20, 540-90+20):
              if data[i] != 0 and data[i] < self.sideLimit*1000:
                data2[i] = data[i]
          elif len(data) == 271:
            s = self.degStep
          else:
            print "Laser bad data size:", len(data)

        arr = [min(i)/1000.0 for i in [islice(data2, start, start+s) for start in range(0,len(data2),s)]]
        if len(arr) > 0:
          viewlog.dumpSharps( robot.localisation.pose(), arr ) 


def roughness0( scan, radius = 5 ):
  "convert scan via roughness function - brute force first"
  res = [0]*radius
  for i in range(radius,len(scan)-radius):
    sum3 = 0
    prev = scan[i-radius]
    for j in range(-radius,radius+1):
      sum3 += (prev-scan[i+j])**2
      prev = scan[i+j]
    res.append( min(sum3,20000) )
  res.extend( [0]*radius )
  return res

def roughness( scan, radius = 5 ): #TODO: Optimize, if possible.
  "convert scan via roughness function"
  res = [0]*radius
  s = 0
  for i in range(2*radius):
    s += (scan[i] - scan[i+1]) ** 2
  res.append( min(s,30000) )
  for i in range(radius+1, len(scan)-radius): # TODO: <-- This loop takes too much time. Note: Pre-caching the squared differences does not help.
    s += (scan[i+radius]-scan[i+radius-1])**2 - (scan[i-radius]-scan[i-radius-1])**2
    res.append( min(s,30000) )
  res.extend( [0]*radius )
#  print len(res)
  return res

def analyseLaser0( filename ):
  prev = None
  for scan in SourceLogger( None, filename ).generator():
    patch = scan[265:275]
    if prev:
      diff = [x-y for (x,y) in zip(prev,patch)]
      print diff
    prev = patch


def median( scan ):
  "return average value with step 10"
  res = []
  for i in range(0,len(scan)-10,10):
    sum = 0
    sum2 = 0
    sum3 = 0
    prev = scan[i]
    for j in range(10):
      sum += scan[i+j]
      sum2 += scan[i+j]*scan[i+j]
      sum3 += (prev-scan[i+j])**2
      prev = scan[i+j]
    avr = int(sum/10)
    avr2 = int(sum2/10)
    for j in range(10):
#      res.append( avr )
#      res.append( int(math.sqrt(avr2-avr*avr)) )
      res.append( min(sum3,10000) )
  res.append( avr ) # extra reading :(
  return res


def analyseLaser( filename ):
  all = []

  scan = []
  for i in range(541):
    angle = (i-270)/2.0
    if math.fabs(angle) < 80:
      s = 2226.0 / math.cos(math.radians(angle))
    else:
      s = 0
    scan.append(s)
  all.append(scan)

  start = 1136/2
  i = 0
  for scan in SourceLogger( None, filename ).generator():
    if i >= start:
      all.append( [s+(i-start)*1000 for s in scan] )
#      all.append( [s+(i-start)*1000 for s in roughness(scan)] )
      if i == start:
        all.append( median(scan) )
#        all.append( roughness(scan) )
    i += 1
    if i >= start+5:
      break
  for i in range(len(all[0])-1,-1,-1): # revert so Excel view is left to right
    s = ""
    for a in all:
      s += "%d\t" % a[i]
    print s


def findBestLine( arr ):
  "find best fitting line in array of points (brute force)"
  bestCount = 0
  bestLine = None
  for a in arr:
    for b in arr:
      if a != b:
        line = Line(a,b)
        count = 0
        for c in arr:
          if math.fabs( line.signedDistance(c) ) < 0.05:
            count += 1
        if count > bestCount:
          print count, (a, b)
          bestCount = count
          bestLine = line
  return bestLine

def testMaxMatchingLine( filename ):
  "find longest line in the scan --- brute force for PC, if it has any sense"
  for scan in SourceLogger( None, filename ).generator():
    assert( len(scan) == 541 ) # TODO failures or different counts
    arr = [(d*math.cos(math.radians(a/2.0))/1000.0, d*math.sin(math.radians(a/2.0))/1000.0) 
        for d,a in zip(scan, range(-270,270+1,1))
        if d > 0]  # ignore timeouts, so note that number of points does not have to be 541 any more
    break
  line = findBestLine( arr )
  for c in arr:
    if math.fabs( line.signedDistance(c) ) < 0.05:
      print "%f\t%f" % c





class ObstacleAvoidance:
  def __init__( self, driver, verbose = False ):
    self.driver = driver
    self.collision = False
    self.handlingCollision = False
    self.verbose = verbose
    self.radius = 1.0
    self.offset = 0 
    self.generator = None
    self.virtualBumper = 0

  def clearCollisionG( self ):
    print "DETECTOR RESTORED"
    self.collision = False
    self.handlingCollision = False
    self.driver.robot.beep = 0
    yield 0, 0

  def updateExtension( self, robot, id, data ):
    if id == 0x80:
      # virtual bumper
      if math.fabs((robot._rampLastLeft+robot._rampLastRight)/2) > 0.2 and math.fabs(robot.currentSpeed) < 0.01:
        self.virtualBumper += 1
        if self.virtualBumper > 40:
          print "VIRTUAL BUMPER", (robot._rampLastLeft+robot._rampLastRight)/2, robot.currentSpeed
          self.collision = True
      else:
        self.virtualBumper = 0

    if id == 'laser' and len(data) > 0:
      # handle 0 = timeout and find min
      near = 10.0
      for x in islice(data, 270-90, 270+90):
        if x != 0 and x < near*1000:
          near = x/1000.0
      # TODO for 1m, slow down and define alternative route
      if near < 0.5:
        self.collision = True

      step = 10
      data2 = [x == 0 and 10000 or x for x in data]
      arr = [min(i)/1000.0 for i in [islice(data2, start, start+step) for start in range(0,len(data2),step)]]
      arr.reverse()

      limit = self.radius
      center = len(arr)/2 + self.offset
      center = min(max(center, 0), len(arr)-1)
      i = 0
      while i < center:
        if arr[center - i] < limit:
          break
        i += 1
      left = i # arr is already reversed
      i = 0
      while i + center < len(arr):
        if arr[center + i] < limit:
          break
        i += 1
      right = i
      s = ''
      for i in arr:
        s += (i<1.0 and 'x' or ' ')
      s = "".join( list(s)[:center] + ['C'] + list(s)[center:] )
      if self.verbose:
        print "'" + s + "'"       
      
      if left < 5 or right < 5:
        # wide row & collision ahead
        if self.verbose:
          print left, right, left+right
        if left < right:
          self.offset += 5
        else:
          self.offset -= 5
        self.directionAngle = math.radians( -self.offset*step/2.0 ) 
        pose = robot.localisation.pose()
        goal = combinedPose( (pose[0], pose[1], pose[2]+self.directionAngle), (self.radius, 0, 0) )
        if not self.collision:
          self.generator = self.driver.followLineG( Line( pose, goal ) )
      elif left >= 10 or right > 10:
        self.offset = 0

    if id == 0x186: 
      assert( len(data) )
      sonar = (data[1]*256 + data[0])*340.0/2.0/1000000.0
      if sonar < 0.5:
        self.collision = True
#      if self.verbose:
#        print "sonar %.2f" % sonar

    if id == 'camera':
      if self.verbose:
        print data[1]

#  def speedController( self, speedL, speedR ):
#    # TODO change interface of speed limit controllers to speed/angSpeed
#    if self.collision:
#      return 0.0, 0.0
#    return speedL, speedR

  def verifyCommand( self, robot, (speed, angularSpeed) ):
 #   print self.collision
    if self.collision and not self.handlingCollision:
      print "COLLISION GENERATOR ON"
      robot.beep = 1
      self.handlingCollision = True
      self.generator = self.driver.multiGen( [
          self.driver.stopG(), 
          self.driver.goStraightG(-0.5),
          self.driver.turnG(math.radians(90)),
          self.clearCollisionG(),
          self.driver.goStraightG(1.0) ] )
    if self.generator != None:
      try:
        return self.generator.next()
      except StopIteration:
        self.generator = None
    return (speed, angularSpeed)


if __name__ == "__main__":
  import sys
  if len(sys.argv) < 2:
    print __doc__
    sys.exit(-1)
#  analyseLaser( sys.argv[1] )
  testMaxMatchingLine( sys.argv[1] )
 
