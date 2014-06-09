#!/usr/bin/python
"""
  Field Robot Event - originally written in 2010, revised 2012
"""
# 2012 TODO list:
# x- integrate launcher
# - detect and handle blocked road
#   - go off-center for more reliable turn-in-place
# - red/blue for Task1 used for left/righ strategy
# - improve end-of-row turns
# - improve rows counting
# - test speed limits (maxSpeed = 0.7 from 2010 is probably the limit)
# - detect ordered rose
# - navigate to it

# x- use PDO for hand up/down
# x- use feed-back for heavy light objects (PID??)


import sys
import math
import itertools

from can import CAN, ReplyLog, ReplyLogInputsOnly
from localisation import SimpleOdometry, KalmanFilter
from driver import Driver, normalizeAnglePIPI, angleTo
from robot import SourceLogger

from eduro import EmergencyStopException, emergencyStopExtension
from eduromaxi import EduroMaxi, buildRobot
import starter
import viewlog

from simulator import Simulator

from route import Convertor
from line import distance, Line
from ray_trace import combinedPose

from viewlog import viewLogExtension, viewCompassExtension
from camera import img2xy

from hand import setupHandModule, handUp, handDown

BALL_SIZE_LIMIT = 100

# test of line counter
def perpDist( d1, d2, alpha ):
  "nearest dist for two separate measurements with given alpha angle difference"
  d = math.sqrt(d1*d1 - 2*d1*d2*math.cos(alpha) + d2*d2)
  return abs(d1*d2*math.sin(alpha)/d)

def splitScan( scan, rowsOnLeft, limit = 2000 ):
  "split scan into monotonous sub-segments"
  # TODO in meters??
  ret = []
  last = []
  for x in scan:
    if x < limit:
      if len(last) == 0 or (x < last[-1] and not rowsOnLeft) or (x > last[-1] and rowsOnLeft):
        last.append( x )
      else:
        if len(last) > 2:
          ret.append( last )
        last = [x]
  if len(last) > 2:
    ret.append( last )
  return ret



# TODO move to robot.py (?)
def compassHeading( rawCompass ):
  "convert raw compass data to heading in radians"
  return math.radians((900-rawCompass)/10.0)

def sprayer( robot, left, right ):
  cmd = 0
  if left:
    cmd += 1
  if right:
    cmd += 2
#  robot.can.sendData( 0x206, [cmd] ) 
  robot.can.sendData( 0x20C, [cmd] ) 

def dumpLaserGeometry():
  # TODO len(data) a step
  geo = [(0,0,math.radians(angle)) for angle in range(-135, 136, 5)]
  print "geo", len(geo)
  viewlog.dumpSharpsGeometry( geo )

def viewLaserExtension( robot, id, data ):
  if id == 'laser':
    if robot.localisation:
      s = 10
      arr = [min(i)/1000.0 for i in [itertools.islice(data, start, start+s) for start in range(0,len(data),s)]]
#      print "arr", len(arr)
#      arr = [x > 1.0 and 10.0 or x for x in arr]
#      arr = [x < 0.3 and 10.0 or x for x in arr]
      viewlog.dumpSharps( robot.localisation.pose(), arr ) 

class LaserRow:
  def __init__( self, radius = 1.0, minSize = 0.3, verbose = False, rowHeading = None ):
    self.radius = radius
    self.minSize = 0.3
    self.verbose = verbose
    self.rowHeading = rowHeading
    self.step = 10 # laser grouping
    self.reset()

  def reset( self, pose = (0,0,0), offsetDeg=0 ):
    print "RESET ROW (%0.2f, %0.2f, %0.1f), offset=" % (pose[0], pose[1], math.degrees(pose[2])), offsetDeg 
    viewlog.dumpBeacon( pose[:2], color=(128,128,128) )
    self.preference = None
    self.center = None
    if self.rowHeading:
      self.directionAngle = normalizeAnglePIPI(self.rowHeading-pose[2])
      if abs(self.directionAngle) > math.radians(90):
        self.directionAngle = normalizeAnglePIPI(self.rowHeading-pose[2]+math.radians(180))
      if self.verbose:
        print "RESET_DIFF %.1f" % math.degrees(self.directionAngle)
    else:
      self.directionAngle = 0.0 # if you do not know, go ahead
    goal = combinedPose( (pose[0], pose[1], pose[2]+self.directionAngle), (self.radius, 0, 0) )
    self.line = Line( pose, goal )
# self.line = Line( (0.0, 0.0), (1.0, 0.0) )
    self.newData = False
    self.endOfRow = False
    self.cornLeft = 10.0 # far
    self.cornRight = 10.0
    self.collisionAhead = 10.0,10.0,False # far (wide, narrow, override)
    self.minGapSize = 3


  def updateExtension( self, robot, id, data ):
    if id == 'remission' and len(data) > 0:
      pass # ignored for FRE 2014

    if id == 'laser' and len(data) > 0:
      step = 10
      data2 = [x == 0 and 10000 or x for x in data]
      arr = [min(i)/1000.0 for i in [itertools.islice(data2, start, start+step) for start in range(0,len(data2),step)]]
      arr.reverse()

      limit = self.radius
      danger = False
      prevCenter = self.center
      if self.center != None:
#        if center > 0 and center < len(arr)-1 and arr[center] < limit and arr[center-1] < limit and arr[center+1] < limit:
        if self.center > 0 and self.center < len(arr)-1 and arr[self.center] < limit:
          print "!!!DANGER!!! PATH BLOCKED!!!", self.center
          danger = True
      if self.center==None or danger:
        # offset is not set - find nearest
        if self.center == None:
          self.center = len(arr)/2
        if arr[self.center] < limit:
          i = 0
          while i < self.center:
            if arr[self.center - i] >= limit:
              break
            i += 1
          left = i
          i = 0
          while i + self.center < len(arr):
            if arr[self.center + i] >= limit:
              break
            i += 1
          right = i
          print "HACK", left, right
          if left < right:
            self.center += -left
          else:
            self.center += right

      i = 0
      if self.center < 0 or self.center >= len(arr):
        self.center = prevCenter
      if self.center == None:
        self.center = len(arr)/2
      while i < self.center:
        if arr[self.center - i] < limit:
          break
        i += 1
      left = i # arr is already reversed
      i = 0
      while i + self.center < len(arr):
        if arr[self.center + i] < limit:
          break
        i += 1
      right = i
      if self.verbose:
        if danger and left+right < self.minGapSize:
          print "MIN GAP SIZE danger:", left+right
        s = ''
        for i in arr:
          s += (i < 0.5 and 'X' or (i<1.0 and 'x' or (i<1.5 and '.' or ' ')))
        s = "".join( list(s)[:self.center] + ['C'] + list(s)[self.center:] )
        print "'" + s + "'"      
      if left+right <= 17: # TODO limit based on minSize, radius and sample angle
        self.center += (right-left)/2
        offset = self.center-len(arr)/2
        self.directionAngle = math.radians( -offset*step/2.0 )
        self.collisionAhead = min(arr[54/3:2*54/3]), min(arr[4*54/9:5*54/9]), (left+right < 3)
        if False: #self.verbose:
          if self.collisionAhead[0] < 0.25:
            print "!!! COLISSION AHEAD !!!", self.collisionAhead
          else:
            print "free space", self.collisionAhead
      elif left < 3 or right < 3:
        # wide row & collision ahead
        print left, right, left+right
        offset = self.center-len(arr)/2
        if left < right:
          offset += 3
        else:
          offset -= 3
        self.directionAngle = math.radians( -offset*step/2.0 )
      else:
#        if self.verbose:
#          print "OFF", left, right, left+right, robot.compass
        if self.rowHeading:
          self.directionAngle = normalizeAnglePIPI(self.rowHeading-robot.localisation.pose()[2])
          if abs(self.directionAngle) > math.radians(90):
            self.directionAngle = normalizeAnglePIPI(self.rowHeading-robot.localisation.pose()[2]+math.radians(180))
#          if self.verbose:
#            print "DIFF %.1f" % math.degrees(self.directionAngle)
        else:
          self.directionAngle = 0.0 # if you do not know, go ahead
        if left >= 17 and right >= 17:
          self.endOfRow = True
          if self.verbose:
            print "laser: END OF ROW"

      pose = robot.localisation.pose()
      goal = combinedPose( (pose[0], pose[1], pose[2]+self.directionAngle), (self.radius, 0, 0) )
      self.line = Line( pose, goal )
      self.newData = True
      self.cornLeft = min(arr[5:9])
      self.cornRight = min(arr[40:44])
#      print "corn", self.cornLeft, self.cornRight, arr[5:9], arr[40:44]



class CameraRow:
  def __init__( self, verbose = False ):
    self.preference = None
    self.directionAngle = 0.0
    self.verbose = verbose
    self.line = Line( (0.0, 0.0), (1.0, 0.0) )
    self.newData = False
    self.endOfRow = False
    self.lastCamera = []
    self.counter = 0

  def updateExtension( self, robot, id, data ):
    if id == 0x80:
      self.counter += 1
      if len(self.lastCamera)> 0:
#        print self.counter, self.lastCamera
        cmd = self.lastCamera[0]
        if self.counter >= cmd[0]:
          self.lastCamera = self.lastCamera[1:]
          if cmd[1] or cmd[2]:
            robot.beep  = 1
          else:
            robot.beep  = 0
          sprayer( robot, cmd[2], cmd[1] ) # swapped due to wrong wiring
    if id == 'camera':
      cc = [int(x) for x in data[0].split()]
      leftPip = (cc[0] > BALL_SIZE_LIMIT )
      rightPip = (cc[1] > BALL_SIZE_LIMIT )
      if leftPip or rightPip:
        print "WEED:", leftPip, rightPip
        self.lastCamera.append( (self.counter + 0, leftPip, rightPip ) )
      else:
        self.lastCamera.append( (self.counter + 0+4, False, False) )



class FieldRobot:
  rowWidth = 0.75
  rowPotsWidth = 0 #0.45
  def __init__( self, robot, configFilename, verbose = False ):
    self.robot = robot
#    self.robot.fnSpeedLimitController = [self.robot.pauseSpeedFn] 
    self.robot.fnSpeedLimitController = [] 
    self.robot.addExtension( emergencyStopExtension )
#    self.robot.attachGPS()
    self.robot.attachLaser( remission=True )
    self.robot.laser.stopOnExit = False  # for faster boot-up
    self.robot.attachCamera( cameraExe = "../robotchallenge/rc" ) # TODO what was used?!
    self.robot.attachHand()
#    self.robot.attachRFID()
    self.robot.rfidData = None # hack 2013
    self.robot.gpsData = None
    self.driver = Driver( self.robot, maxSpeed = 0.7, maxAngularSpeed = math.radians(60) )
#    self.robot.localisation = KalmanFilter() # needed for better direction handling
    self.robot.localisation = SimpleOdometry()
    self.verbose = verbose
    self.configFilename = configFilename
    self.rowHeading = None

  def waitForStart( self ):
    print "Waiting for start cable insertion ..."
    while self.robot.startCableIn is None or not self.robot.startCableIn:
      self.robot.setSpeedPxPa( 0.0, 0.0 )
      self.robot.update()
    print "READY & waiting for start ..."
    while self.robot.startCableIn is None or self.robot.startCableIn:
      self.robot.setSpeedPxPa( 0.0, 0.0 )
      if self.robot.laserData:
        self.robot.toDisplay = 'O'
        if self.robot.remissionData:
          self.robot.toDisplay = 'R'
      else:
        self.robot.toDisplay = '-'
      if self.robot.rfidData:
        self.robot.toDisplay += str( self.robot.rfidData[0] % 10 )
      else:
        self.robot.toDisplay += '-'

      self.robot.update()
    print "!!! GO !!!" 

  def pickPot( self ):
    "pick pot with rose"
    # make space for the fork-lift
    print "LOADING POT"
    self.driver.turn( math.radians(-50), radius = 0.40, angularSpeed=math.radians(20) )
    handDown(self.robot,timeout=None)
    self.driver.turn( math.radians(50), angularSpeed=math.radians(20) )
    self.driver.turn( math.radians(20), radius = -0.28, angularSpeed=math.radians(20) )    
    handUp(self.robot,timeout=None)
    self.driver.turn( math.radians(-20), radius = 0.28, angularSpeed=math.radians(20) )
    self.driver.turn( math.radians(-50), angularSpeed=math.radians(20) )
    self.driver.turn( math.radians(50), radius = -0.40, angularSpeed=math.radians(20) )
    print "POT LOADED"


  def ver2( self, code, detectWeeds = True, detectBlockedRow = True ):
    print "Field Robot - LASER & CAMERA"

    try:
      # start GPS sooner to get position fix
#      self.robot.gps.start()
      self.robot.laser.start()
#      self.robot.rfid.start()
      self.waitForStart()
      if not self.robot.switchBlueSelected:
        print "RED -> mirroring code directions!!!"
        code = [-x for x in code]
        print code

      if self.robot.compass:
        self.rowHeading = compassHeading(self.robot.compass)
      self.robot.camera.start()
      laserRow = LaserRow( verbose = self.verbose, rowHeading = self.robot.localisation.pose()[2] )
      self.robot.addExtension( laserRow.updateExtension )
      cameraRow = CameraRow( verbose = self.verbose )
      self.robot.addExtension( cameraRow.updateExtension )
      angularSpeed = 0.0
      self.robot.maxSpeed = 0.2
      speed = 0 # wait for first camera image (maybe beep?? if no route to host)
      startTime = self.robot.time
      row = laserRow
      row.reset( offsetDeg=None ) # just for test, try to find the gap (kitchen test)
#      row = cameraRow
      for action in code:
        print "=================  ACTION ", action, "================="
        print "battery:", self.robot.battery
        while not row.endOfRow:
#          sprayer( self.robot, True, True )
          for (speed, angularSpeed) in self.driver.followLineG( row.line ):
            if self.robot.laserData:
              self.robot.toDisplay = 'OK'
            else:
               self.robot.toDisplay = '--'
               speed, angularSpeed = 0.0, 0.0            
#            if detectBlockedRow and row.collisionAhead[1] < 0.25:
            if detectBlockedRow and row.collisionAhead[2]:
              print "---------- COLLISON -> TURN 180 -------------"
              self.robot.beep = 1
              self.driver.stop()
#              self.driver.goStraight( -0.5 )
              self.driver.turn( math.radians(90), radius = 0.12, angularSpeed=math.radians(120), withStop=False )
              self.driver.turn( math.radians(90), radius = -0.12, angularSpeed=math.radians(120) )
              self.robot.beep = 0
#              self.driver.turn( math.radians(180), radius = 0.075 )
#              self.driver.turn( math.radians(180), radius = 0.0 )
#              self.driver.turn( math.radians(20), radius = 0.37, angularSpeed=math.radians(20) )
#              self.driver.turn( math.radians(70), radius = 0.08, angularSpeed=math.radians(20) )
#              self.driver.turn( math.radians(70), radius = -0.08, angularSpeed=math.radians(20) )
#              self.driver.turn( math.radians(20), radius = -0.37, angularSpeed=math.radians(20) )
              self.robot.setSpeedPxPa( 0.0, 0.0 )
              row.reset( self.robot.localisation.pose(), offsetDeg = None ) # instead of turning shift search
            else:
#              if detectBlockedRow and (row.collisionAhead[0] < 0.5 or row.collisionAhead[1] < 0.7):
#                if self.verbose:
#                  print "!!SLOW DOWN!!"
#                speed, angularSpeed = speed/2.0, angularSpeed/2.0 # slow down
              self.robot.setSpeedPxPa( speed, angularSpeed )
            self.robot.update()
            if row.newData:
              row.newData = False
              break
#          sprayer( self.robot, False, False )

        # handle action after row termination
        self.robot.beep = 1
        self.driver.stop()
        self.robot.beep = 0
        if action == 0:
          self.driver.goStraight( 0.2 )
          prevAngle = self.robot.localisation.pose()[2]
          if not self.driver.turn( math.radians(180), timeout=7.0 ):
            self.driver.goStraight( -0.2 )
            currAngle = self.robot.localisation.pose()[2]
            print "TIMEOUT", math.degrees(normalizeAnglePIPI(math.radians(180)-currAngle+prevAngle))
            if not self.driver.turn( normalizeAnglePIPI(math.radians(180)-currAngle+prevAngle), timeout=5.0 ):
              currAngle = self.robot.localisation.pose()[2]
              print "TIMEOUT2 - GIVING UP", math.degrees(normalizeAnglePIPI(math.radians(180)-currAngle-prevAngle))            
        elif action == -1:
          self.driver.turn( math.radians(160), radius = 0.75/2.0, angularSpeed=math.radians(40) )
        elif action == 1:
          self.driver.turn( math.radians(-160), radius = 0.75/2.0, angularSpeed=math.radians(40) )
        else:
          if action < 0:
            self.driver.turn( math.radians(90), radius = self.rowWidth/2.0, angularSpeed=math.radians(40) )
          else:
            self.driver.turn( math.radians(-90), radius = self.rowWidth/2.0, angularSpeed=math.radians(40) )
#          self.driver.goStraight( (math.fabs(action)-1) * (self.rowWidth+self.rowPotsWidth)+self.rowPotsWidth )
          self.crossRows( math.fabs(action)-1, rowsOnLeft = (action < 0) )
          if action < 0:
            self.driver.turn( math.radians(90), radius = self.rowWidth/2.0, angularSpeed=math.radians(40) )
          else:
            self.driver.turn( math.radians(-90), radius = self.rowWidth/2.0, angularSpeed=math.radians(40) )

        # clear flag for detection
#        row.reset( self.robot.localisation.pose() )
        row.reset( self.robot.localisation.pose(), offsetDeg=None ) # init with search for center
      self.robot.setSpeedPxPa( 0.0, 0.0 )
      self.robot.update() 
    except EmergencyStopException, e:
      print "EmergencyStopException"
    sprayer( self.robot, 0, 0 )         
    print "battery:", self.robot.battery
    self.robot.camera.requestStop()
#    self.robot.gps.requestStop()
#    self.robot.rfid.requestStop()
    self.robot.laser.requestStop()

  def testSpreyer( self ):
    sprayer( self.robot, True, False )
    for i in range(50):
      self.robot.update()
    sprayer( self.robot, False, True )
    for i in range(50):
      self.robot.update()
    sprayer( self.robot, False, False )

  def testPickPot( self ):
    try:
      # start GPS sooner to get position fix
      self.robot.gps.start()
      self.robot.laser.start()
      self.waitForStart()
      self.robot.camera.start()
      self.pickPot()
    except EmergencyStopException, e:
      print "EmergencyStopException"
    self.robot.camera.requestStop()
    self.robot.gps.requestStop()
    self.robot.laser.requestStop()

  def testA( self ):
    self.driver.turn( math.radians(-20), radius = 0.28 )
    self.driver.turn( math.radians(-20) )
    handUp(self.robot,timeout=None)
    self.driver.turn( math.radians(40), radius = -0.40 )
  def testB( self ):
    self.driver.turn( math.radians(-40), radius = 0.40 )
    handDown(self.robot,timeout=None)
    self.driver.turn( math.radians(20) )
    self.driver.turn( math.radians(20), radius = -0.28 )
  def testX( self ):
    for i in xrange(10):
      self.testA()
      self.testB()
      handUp(self.robot,timeout=None)

  def crossRows( self, num, rowsOnLeft ):
    "follow N lines without turns"
    self.driver.goStraight( num * (self.rowWidth+self.rowPotsWidth)+self.rowPotsWidth )
    return

    i = 0
    prevDist = None
    for cmd in self.driver.goStraightG( num * (self.rowWidth+self.rowPotsWidth)+self.rowPotsWidth ):
      self.robot.setSpeedPxPa( *cmd ) 
      self.robot.update()
      if i % 10 == 0:
        if rowsOnLeft:
          tmp = [int(x) for x in self.robot.laserData][270:450:10]
        else:
          tmp = [int(x) for x in self.robot.laserData][90:270:10]
        ss = splitScan( tmp, rowsOnLeft )
        distArr = [int(perpDist(a[0], a[-1], (len(a)-1)*math.radians(5)))/1000. for a in ss]
        print ss, distArr
        if prevDist != None and len(distArr) > 0 and prevDist < min(distArr):
          num -= 1
          if num == 0:
            break
        if len(distArr) > 0:
          prevDist = min(distArr)
      i += 1


  def __call__( self ):
    print "RUNNING:", self.configFilename
    if self.configFilename.startswith("cmd:"):
      return eval( self.configFilename[4:] )
#    return self.testX()
#    return self.pickPot()
#    for i in xrange(10):
#      handDown( self.robot, timeout=None )
#      handUp( self.robot, timeout=None )
#    return self.testPickPot()
#    return self.ver2([-1,1]*10, detectWeeds = False, detectBlockedRow = False)  # Task1
#    return self.ver2( [0,-1,0,-1,2], detectWeeds = False, detectBlockedRow = True ) # Task2
#    return self.ver2([-1,1]*10, detectWeeds = True, detectBlockedRow = False)  # Task3
    return self.ver2( [-2,2,-2,2], detectWeeds = False, detectBlockedRow = True ) # Task2

from eduromaxi import EduroMaxi
import launcher
if __name__ == "__main__": 
  launcher.launch(sys.argv, EduroMaxi, FieldRobot, configFn=setupHandModule)

