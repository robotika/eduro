#!/usr/bin/python
"""
  SICK Robot Day 2010 main program
    usage:
         ./sickday.py <code> [--file <log file> [F|FF]]
"""

import sys
import math
import os
import random 
from itertools import izip, islice

try: # Use Psyco, if available.
  import psyco
  psyco.full()
except ImportError:
  pass

from eduro import EmergencyStopException
from eduromaxi import EduroMaxi, buildRobot, cameraDataExtension
from driver import Driver, normalizeAnglePIPI, angleTo
from localisation import SimpleOdometry, KalmanFilter

from can import CAN, ReplyLog, ReplyLogInputsOnly 

import viewlog
from viewlog import viewLogExtension, viewCompassExtension, viewPoseExtension
from laseranalysis import LaserViewer, ObstacleAvoidance, roughness
from vfh import VFH

from ray_trace import combinedPose
from line import distance, Line
from route import loadLatLonPts, Convertor

import starter
from hand import setupHandModule

def computeLoadManeuver( minDistL, frontDist, minDistR ):
  "compute rotation and backup distance as last maneuver near the ring"
  print "minDist", minDistL, frontDist, minDistR
  laserZero = 0.045  
  laserOffset = 0.11
  bowlOffset = 0.31
  wallRingCenterDist = 0.25
  if frontDist is None or min(minDistL,minDistR) >= frontDist:
    # "asserts" for missing frontDist or smaller or equal to minDist - 
    toTurn = math.radians(180)
    toMove = 0.0
  else:
    minDistL += laserZero
    minDistR += laserZero
    frontDist += laserZero
    minDist = min(minDistL, minDistR)
    alpha = math.acos(minDist/frontDist)
    A = math.sqrt(frontDist**2 - minDist**2) + abs(math.sin(alpha)*laserOffset)
    B = minDist-wallRingCenterDist + math.cos(alpha)*laserOffset
    print "AB", A, B
    beta = math.atan2( A, B )
    if minDistR < minDistL:
      # i.e. wall on mine right side
      print math.degrees(alpha), math.degrees(beta)
      toTurn = math.radians(-180) + (beta - alpha)
    else:
      toTurn = math.radians(180) - (beta-alpha)
    toMove = bowlOffset - math.hypot( A, B )
  return toTurn, toMove


class ViewCameraExtension:
  def __init__( self, absPath ):
    self.absPath = absPath

  def viewCameraExtension( self, robot, id, data ):
    #TODO: Except of the absPath vers sys.argv[2][:-18] difference, this is same as viewlog.viewCameraExtension
    if id == 'camera' and len(data)>=2 and data[1] != None:
      viewlog.dumpCamera( self.absPath + os.sep + data[1].split('/')[-1], data[0].strip() ) 


class LaserPath:
  FAR_AWAY = 100000

  "Copy & Paste from FRE2010"
  def __init__( self, verbose = False ):
    self.verbose = verbose
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
    if id == 'laser' and len(data) > 0:
      # Is there an obstacle somewhere very close?
      lbound = len(robot.laserData) / 6
      rbound = len(robot.laserData) * 5 / 6
      # (lbound, rbound) should cover the space ahead of the robot.
      obstacle_threshold = 300 # [mm]
      for x in islice(robot.laserData, lbound, rbound):
        if x > 0 and x < obstacle_threshold:
            self.line = None
            self.newData = True
            self.directionAngle = 0.0
            if self.verbose:
              s = ''
              for start in xrange(0, len(robot.laserData), self.step):
                too_close = False
                for x in islice(robot.laserData, start, start+self.step):
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


def parseCameraData( camData ):
#  return eval( camData[0] )
  # TODO smarter filtering of multiple thresholds
  arr = eval( camData[0] )
  ret = []
  for a in arr:
    ret.extend( a )
  return ret  # merge all results


class DigitFound( Exception ):
  def __init__( self, info ):
    Exception.__init__(self)
    self.info = info

class DigitDetecor:
  def __init__(self, digit):
    self.digit = digit

  def updateExtension(self, robot, id, data):
    if id == 'camera':
      digits = parseCameraData( data )
      for d in digits:
        if d[0] == self.digit:
          raise DigitFound( d )


def zeroCmd():
  "replacement for Driver.stopG()"
  while True:
    yield (0,0) # never ending STOP


class SICKRobotDay2014:
  def __init__( self, robot, code, verbose = False ):
    self.random = random.Random(0).uniform 
    self.robot = robot
    self.verbose = verbose
    self.code = code
    self.robot.attachEmergencyStopButton()
    
    self.robot.attachCamera( cameraExe = "../digits/digits", 
        url = "http://192.168.0.99/image?res=full&x0=352&y0=80&x1=992&y1=592&quality=12&doublescan=0" )
#    self.robot.attachCamera( cameraExe = "../digits/digits" )
    self.robot.addExtension( cameraDataExtension )
    self.robot.attachLaser( pose=((0.14, 0.0, 0.32), (0,0,0)) )
    self.robot.attachLaser( index=2, remission=True, usb=True, 
        pose=((0.19, 0.0, 0.055), tuple([math.radians(x) for x in (0, 180, 0)])), 
        errLog = self.robot.metaLog )
    self.robot.attachBarcodeReader()
    
    self.driver = Driver( self.robot, maxSpeed = 0.5, maxAngularSpeed = math.radians(180) )
    self.robot.localisation = SimpleOdometry()
    
    self.robot.laser.stopOnExit = False  # for faster boot-up


  def run( self ):
    try:
      if getattr( self.robot.laser, 'startLaser', None ):
        # trigger rotation of the laser, low level function, ignore for log files
        print "Powering laser ON"
        self.robot.laser.startLaser() 
      self.robot.waitForStart()
      self.robot.laser.start()  # laser also after start -- it should be already running
      self.robot.laser2.start() 
      self.robot.camera.start()
      self.robot.barcode.start()
      self.robot.localisation = SimpleOdometry()
      while True:
        self.ver0(verbose = self.verbose)      
    except EmergencyStopException, e:
      print "EmergencyStopException"
    self.robot.laser.requestStop()
    self.robot.laser2.requestStop() 
    self.robot.camera.requestStop()
    self.robot.barcode.requestStop()

  def turnLights( self, on=True ):
    cmd = 0
    if on:
      cmd = 1+2
    self.robot.can.sendData( 0x20C, [cmd] ) 
 

  def goToDigit( self, digit, info = None ):
    # prepositions - digit visible and near
    prevMinDist = None
    columnChecked = False
    self.robot.toDisplay = '>' + str(digit)
    oldCam = self.robot.cameraData
    countImages = 0
    countVerified = 0
    for cmd in self.driver.goStraightG( 10.0 ):
      if oldCam != self.robot.cameraData:
        oldCam = self.robot.cameraData
        countImages += 1
        if self.robot.cameraData and self.robot.cameraData[0]:
          info = None
          for d in parseCameraData( self.robot.cameraData ):
            if d[0] == digit:
              info = d
          if info != None:
            countVerified += 1
#        print countVerified, countImages, self.robot.cameraData
      if countImages >= 3 and countVerified < 2:  # TODO tuning params
        return False

      if info:
#        cmd = (cmd[0], math.radians(0.1*(640/2 - info[1][0]))) # x-coord
        cmd = (cmd[0], math.radians(0.1*(640/2 - (info[1][0]+info[1][2]/2.0)))) # x-coord + width/2

      if self.robot.laserData and len(self.robot.laserData) > 0:
        minDist = min( [x for x in self.robot.laserData[ 270-90: 270+90 ]] )/1000.0 # plus/minus 45deg            
#        if prevMinDist != None:
#          print "DIST", prevMinDist-minDist
        if prevMinDist != None and prevMinDist + 0.2 < minDist and prevMinDist < 0.8:
          print "ERROR DIST", prevMinDist, minDist, oldCam
          return False
        prevMinDist = minDist
        if minDist < 1.5:
          break
      self.robot.setSpeedPxPa( *cmd )
      self.robot.update()
    self.approachFeeder( digitHelper=digit )
    return True


  def turnWithWatchdog( self, angle, angularSpeed ):
    timeout = math.fabs(angle/angularSpeed) + 5.0 # for normal operation it looks likt 0.5s is sufficient
    limit = self.robot.time+timeout
    for cmd in self.driver.turnG( angle, angularSpeed = angularSpeed ):
      self.robot.setSpeedPxPa(*cmd)
      self.robot.update()
      if self.robot.time > limit:
        print "TIMEOUT", self.robot.time, timeout
        return False
    return True

  def wait( self, duration ):
    # TODO wait for detection
    print "Waiting ..."
    startTime = self.robot.time
    while startTime + duration > self.robot.time:
      self.robot.setSpeedPxPa( 0, 0 )
      self.robot.update()


  def approachFeeder( self, timeout=60, digitHelper=None ):
    "robot should be within 1m of the feeder"
    print "Approaching Feeder"
    desiredDist = 0.4 #0.2
    countOK = 0
    startTime = self.robot.time
    angularSpeed = 0
    prevPose = None
    prevName = None
    prevLaser = None
    target = None
    frontDist, minDistL, minDistR = None, None, None
    while startTime + timeout > self.robot.time:
      if self.robot.cameraData is not None and len(self.robot.cameraData)> 0 and self.robot.cameraData[0] is not None:
        if prevName != self.robot.cameraData[1]:
#          print prevName, self.robot.localisation.pose()
          prevName = self.robot.cameraData[1]
          if prevPose is None:
            prevPose = self.robot.localisation.pose()
            prevLaser = self.robot.laserData[:]
          arr = eval(self.robot.cameraData[0])
          angularSpeed = None
          for a in arr:
            for digit, (x,y,dx,dy) in a:
              if digit == 'X':
                angularSpeed = (320-(x+dx/2))/100.0
#                print "angularSpeed", math.degrees(angularSpeed), self.robot.cameraData[1], (x,y,dx,dy)
                centerX = 320
                angle = (centerX-(x+dx/2))*0.002454369260617026
                dist = prevLaser[int(angle/2.)+271]/1000.
#                print dist
                t = combinedPose( (prevPose[0], prevPose[1], prevPose[2]+angle), (dist,0,0) )
                target = (t[0],t[1])
                viewlog.dumpBeacon( target, color=(255,128,0) )
                break
          if target is None and digitHelper is not None:
            # if you do not have goal try to re-search old number
            for a in arr:
              for digit, (x,y,dx,dy) in a:
                if digit == 'X':
                  angularSpeed = (320-(x+dx/2))/100.0
                  centerX = 320
                  angle = (centerX-(x+dx/2))*0.002454369260617026
                  dist = prevLaser[int(angle/2.)+271]/1000.
                  t = combinedPose( (prevPose[0], prevPose[1], prevPose[2]+angle), (dist,0,0) )
                  target = (t[0],t[1])
                  viewlog.dumpBeacon( target, color=(255,0,255) )
                  break

          prevPose = self.robot.localisation.pose()
          prevLaser = self.robot.laserData[:]
        if angularSpeed is None:
          angularSpeed = 0

      if target is None or distance( target, self.robot.localisation.pose() ) < 0.3:
        angularSpeed = 0.0
      else:
        pose = self.robot.localisation.pose()
        angularSpeed = normalizeAnglePIPI( angleTo( pose, target) - pose[2] )
#        print "target:", target, math.degrees(angularSpeed)


      if self.robot.laserData == None or len(self.robot.laserData) != 541:
        self.robot.setSpeedPxPa( 0, 0 )
      else:
        minDistR = min([10000]+[x for x in self.robot.laserData[180:541/2] if x > 0])/1000.
        minDistL = min([10000]+[x for x in self.robot.laserData[541/2:-180] if x > 0])/1000.
        minDist = min(minDistL, minDistR)
        frontDist = min([10000]+[x for x in self.robot.laserData[265:-265] if x > 0])/1000.
        self.robot.setSpeedPxPa( min(self.driver.maxSpeed, minDist - desiredDist), angularSpeed )
#        print min(self.driver.maxSpeed, minDist - desiredDist)
#        self.robot.setSpeedPxPa( 0, angularSpeed )
        if abs(minDist - desiredDist) < 0.01:
          countOK += 1
        else:
          countOK = 0
        if countOK >= 10:
          break
      self.robot.update()
    self.robot.setSpeedPxPa( 0, 0 )
    self.robot.update()
    print "done."

    # compute proper rotation and backup distance
    toTurn, toBackup = computeLoadManeuver( minDistL, frontDist, minDistR )
    print "Suggestion: ", math.degrees(toTurn), toBackup
    self.driver.turn( toTurn, angularSpeed = math.radians(20) )
    self.driver.goStraight( toBackup )


  def waitForCode( self, timeout=10 ):
    startTime = self.robot.time
    while startTime + timeout > self.robot.time:
      if self.robot.barcodeData is not None:
        return self.robot.barcodeData[0]
      self.robot.setSpeedPxPa( 0, 0 )
      self.robot.update()


  def goVfh( self, timeout ):
    TOLERATED_MISS = 0.2 # [m]
    ANGULAR_THRESHOLD = math.radians(20) # [rad]
    ANGULAR_SPEED = math.radians(60) # [rad/s]

    ENDURANCE = 3 # Amount of time the robot remains blocked before timing out and moving backwards. [s]
    RETREAT = 0.2 # A retreated distance after endurance times out. [m]

    pathFinder = LaserPath( verbose=False )
    self.robot.addExtension( pathFinder.updateExtension, "PATH" )

    turnRadius = None #1.2
#    vfh = VFH(self.robot, blockingDistance = 0.35, safetyDistance = 0.6, maxRange = 1.4, turnRadius = turnRadius, verbose = False)
    vfh = VFH(self.robot, blockingDistance = 0.2, safetyDistance = 0.3, maxRange = 1.0, turnRadius = None, verbose = False)
    self.robot.addExtension(vfh.updateExtension, "VHF")
    # Navigate roughly in the suggested direction, avoiding obstacles.
    prevDir = 0.0
    goalDir = 0.0

    startTime = self.robot.time
    happyTime = self.robot.time
    while self.robot.time < startTime + timeout:
      dir = vfh.navigate(goalDir, prevDir, pathFinder.obstacles)
      if dir is None or not pathFinder.isTraversable(dir):
        strategy = zeroCmd()
        prevDir = 0.0
        self.robot.toDisplay = 'HH'
      else:
        pose = self.robot.localisation.pose()
        goal = combinedPose( (pose[0], pose[1], pose[2]+dir), (1.0, 0, 0) )
        strategy = self.driver.goToG( goal , TOLERATED_MISS, angleThreshold = ANGULAR_THRESHOLD,  angularSpeed = ANGULAR_SPEED)
        prevDir = dir
        self.robot.toDisplay = 'OK'
        happyTime = self.robot.time

      if self.robot.time - happyTime > ENDURANCE:
        break # does not have sense to wait - try to turn somewhere else ...
        # We are stuck for too long. Let's try to retreat a bit:
        #    1) to show we are alive,
        #    2) to hope the robot gets out of the blocked state.
        # Note: p(bumping backward) <= p(being disqualified for not moving) == 1.0
        pose = self.robot.localisation.pose()
        goal = combinedPose( pose, (-(RETREAT + TOLERATED_MISS), 0, 0) )
        strategy = self.driver.goToG( goal , TOLERATED_MISS, backward = True, angleThreshold = ANGULAR_THRESHOLD,  angularSpeed = ANGULAR_SPEED)
        self.robot.toDisplay = '\/'
        for (speed, angularSpeed) in strategy:
          self.robot.setSpeedPxPa( speed, angularSpeed )
          self.robot.update()
        happyTime = self.robot.time
        continue # TODO maybe replace by return -> give up

      for (speed, angularSpeed) in strategy:
        self.robot.setSpeedPxPa( speed, angularSpeed )
        self.robot.update()
        pose = self.robot.localisation.pose()
#        isThere = math.hypot(pose[0] - lastX, pose[1] - lastY) <= TOLERATED_MISS
#        if isThere or pathFinder.newData:
        if pathFinder.newData:
          pathFinder.newData = False
          break

    self.robot.removeExtension( "VHF" )
    self.robot.removeExtension( "PATH" )


  def cam2waypoint( self, pose, camInfo ):
    "estimate from camera info (x,y,width,height) new waypoint"
    if camInfo is None:
      dir = 0
      dist = 10.0
    else:
      dir = math.radians(0.1* (640/2-(camInfo[1][0]+camInfo[1][2]/2.0)))
      dist = 20.0/float(camInfo[1][2]) # TODO calibrate 
    return combinedPose( (pose[0], pose[1], pose[2]+dir), (dist, 0, 0) )


  def waypoint2dir( self, pose, waypoint ):
    "return direction to waypoint for current pose"
    absAngle = math.atan2(waypoint[1]-pose[1], waypoint[0]-pose[0])
    relAngle = absAngle - pose[2]
    return normalizeAnglePIPI(relAngle)


  def goToVfhDigit( self, digit, timeout, info = None ):
    TOLERATED_MISS = 0.2 # [m]
    ANGULAR_THRESHOLD = math.radians(20) # [rad]
    ANGULAR_SPEED = math.radians(60) # [rad/s]

    ENDURANCE = 3 # Amount of time the robot remains blocked before timing out and moving backwards. [s]
    RETREAT = 0.2 # A retreated distance after endurance times out. [m]

    pathFinder = LaserPath( verbose=False )
    self.robot.addExtension( pathFinder.updateExtension, "PATH" )

    turnRadius = None #1.2
#    vfh = VFH(self.robot, blockingDistance = 0.35, safetyDistance = 0.6, maxRange = 1.4, turnRadius = turnRadius, verbose = False)
    vfh = VFH(self.robot, blockingDistance = 0.2, safetyDistance = 0.3, maxRange = 1.0, turnRadius = None, verbose = False)
    self.robot.addExtension(vfh.updateExtension, "VHF")
    # Navigate roughly in the suggested direction, avoiding obstacles.
    prevDir = 0.0

    waypoint = self.cam2waypoint( self.robot.localisation.pose(), info )
    goalDir = self.waypoint2dir( self.robot.localisation.pose(), waypoint )
    print "GOAL DIR", goalDir

    oldCam = self.robot.cameraData
    countImages = 0
    countVerified = 0

    startTime = self.robot.time
    happyTime = self.robot.time
    while self.robot.time < startTime + timeout:
      if oldCam != self.robot.cameraData:
        oldCam = self.robot.cameraData
        countImages += 1
        if self.robot.cameraData and self.robot.cameraData[0]:
          info = None
          for d in parseCameraData( self.robot.cameraData ):
            # TODO switch to goToDigit for large enough number
            if d[0] == digit:
              info = d
          if info != None:
            countVerified += 1
            waypoint = self.cam2waypoint( self.robot.localisation.pose(), info )
            goalDir = self.waypoint2dir( self.robot.localisation.pose(), waypoint )
            print "GOAL DIR", goalDir, oldCam, info, digit

      dir = vfh.navigate(goalDir, prevDir, pathFinder.obstacles)
      if dir is None or not pathFinder.isTraversable(dir):
        strategy = zeroCmd()
        prevDir = 0.0
        self.robot.toDisplay = 'HH'
      else:
        pose = self.robot.localisation.pose()
        goal = combinedPose( (pose[0], pose[1], pose[2]+dir), (1.0, 0, 0) )
        strategy = self.driver.goToG( goal , TOLERATED_MISS, angleThreshold = ANGULAR_THRESHOLD,  angularSpeed = ANGULAR_SPEED)
        prevDir = dir
        self.robot.toDisplay = 'OK'
        happyTime = self.robot.time

      if self.robot.time - happyTime > ENDURANCE:
        break # does not have sense to wait - try to turn somewhere else ...
        # We are stuck for too long. Let's try to retreat a bit:
        #    1) to show we are alive,
        #    2) to hope the robot gets out of the blocked state.
        # Note: p(bumping backward) <= p(being disqualified for not moving) == 1.0
        pose = self.robot.localisation.pose()
        goal = combinedPose( pose, (-(RETREAT + TOLERATED_MISS), 0, 0) )
        strategy = self.driver.goToG( goal , TOLERATED_MISS, backward = True, angleThreshold = ANGULAR_THRESHOLD,  angularSpeed = ANGULAR_SPEED)
        self.robot.toDisplay = '\/'
        for (speed, angularSpeed) in strategy:
          self.robot.setSpeedPxPa( speed, angularSpeed )
          self.robot.update()
        happyTime = self.robot.time
        continue # TODO maybe replace by return -> give up

      for (speed, angularSpeed) in strategy:
        self.robot.setSpeedPxPa( speed, angularSpeed )
        self.robot.update()
        pose = self.robot.localisation.pose()
#        isThere = math.hypot(pose[0] - lastX, pose[1] - lastY) <= TOLERATED_MISS
#        if isThere or pathFinder.newData:
        if pathFinder.newData:
          pathFinder.newData = False
          break

    self.robot.removeExtension( "VHF" )
    self.robot.removeExtension( "PATH" )


  def goToCenterArea( self ):
    for cmd in self.driver.goStraightG( 10.0 ):
      # TODO parse laser data to find "local minima island"
      # TODO VFH for collision avoidance
      if self.robot.laserData == None or len(self.robot.laserData) != 541:
        self.robot.setSpeedPxPa( 0, 0 )
      else:
        minDist = min([10000]+[x for x in self.robot.laserData[180:-180] if x > 0])/1000.
        if minDist < 2.0:
          break
#        print minDist
#        self.robot.setSpeedPxPa( min(self.driver.maxSpeed, minDist - desiredDist), angularSpeed )
      self.robot.setSpeedPxPa( *cmd )
      self.robot.update()
#    self.robot.beep = 1
    self.driver.stop()


  def ver0( self, verbose=False ):
    # follow each number separaterly
    print "ver0", self.code, self.robot.battery
    gameStartTime = self.robot.time
    while True:
      self.goToCenterArea()
      self.approachFeeder()
      self.turnLights( on=True )
      digit = self.waitForCode()
      self.turnLights( on=False )
      if digit is None:
        self.driver.goStraight(0.2)
        # shake
        while self.robot.barcodeData is None:
          acc = self.robot.maxAngularAcc
          self.robot.maxAngularAcc = acc*2
          self.driver.turn( angle=math.radians(45), angularSpeed=math.radians(180) )
          self.driver.turn( angle=math.radians(-90), angularSpeed=math.radians(180) )
          self.driver.turn( angle=math.radians(45), angularSpeed=math.radians(180) )
          self.robot.maxAngularAcc = acc
        digit = self.robot.barcodeData[0]
      if digit is not None:
        digitMissionCompleted = False
        while not digitMissionCompleted:
          print "LOOKING FOR digit =", digit
          self.robot.toDisplay = '>' + str(digit)
          det = DigitDetecor( digit )
          self.robot.addExtension( det.updateExtension, "DIGI" )
          try:
            while True:
              # "random" walk with purpose to find the number
              self.goVfh( self.random(2.0, 30.0) )
              self.turnWithWatchdog( math.radians(self.random(-180, 180)), angularSpeed = math.radians(20) )
          except DigitFound, e:
            print "FOUND", digit, e.info
            self.robot.removeExtension( "DIGI" ) # so we won't get other exceptions
            if self.goToDigit( digit, info=e.info ):
              digitMissionCompleted = True
              print "DIGIT", digit, "COMPLETED", self.robot.time-gameStartTime
              self.turnLights(on=True)
              self.wait( 10.0 )
              self.turnLights(on=False)
            else:
              print "DIGIT", digit, "FAILURE -> repeat"
              self.driver.stop()
              self.driver.goStraight(-0.2)
              self.turnWithWatchdog( math.radians(self.random(10, 180)), angularSpeed = math.radians(40) )
    print 'Game over.', self.robot.battery
    for k in xrange(10):
      self.robot.setSpeedPxPa(0.0, 0.0)
      self.robot.update()
    raise EmergencyStopException() # TODO: Introduce GameOverException as in Eurobot


  def __call__( self ):
    print "RUNNING:", self.code
    if self.code.startswith("cmd:"):
      return eval( self.code[4:] ) 
    return self.run()


if __name__ == "__main__": 
  # code = sys.argv[1]  ... i.e. 123456789 or 876543210 or some other combinations for test
  from eduromaxi import EduroMaxi
  import launcher
  launcher.launch(sys.argv, EduroMaxi, SICKRobotDay2014, configFn=setupHandModule)

