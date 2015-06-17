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
import datetime

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
from camera import img2xy, timeName

from hand import setupHandModule, handUp, handDown

#BALL_SIZE_LIMIT_MIN = 100
#BALL_SIZE_LIMIT_MAX = 150
BALL_SIZE_LIMIT_MIN = 1400
BALL_SIZE_LIMIT_MAX = 1500000

MIN_GAP_SIZE = 4 #3 #5
MAX_GAP_SIZE = 13 #17
SLOW_DOWN_ANGLE = math.radians(15.0)

RED_ALERT = 100

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

g_ballsFile = None
g_numBalls = 0
def reportBall( coord ):
  global g_ballsFile
  global g_numBalls
  if g_numBalls >= 5 or coord == None:
    return
  g_numBalls += 1
# ['$PTNL', 'PJK', '160714.00', '061714', '+5746405.893', 'N', '+686285.539', 'E', '2', '08', '2.8', 'EHT+123.664', 'M']
  assert len(coord) == 13, coord
  assert coord[0] == '$PTNL', coord[0]
  assert coord[1] == 'PJK', coord[1]
  t = coord[2].split('.')[0]
  d = coord[3]
  if g_ballsFile == None:
    g_ballsFile = open( "logs/FRE-Task3_EduroTeam_" + d[4:]+d[:2]+d[2:4] + "_" + t + ".txt", "w")
    g_ballsFile.write( "Team Name: Eduro Team\n" )
    g_ballsFile.write( "Date and Time: %s.%s.20%s %s:%s:%s\n" % (d[2:4], d[:2], d[4:], t[:2], t[2:4], t[4:] ) )  # DD.MM.YYYY hh:mm:ss
  g_ballsFile.write( ",".join(coord[4:8]) + "\n" ) # +5395463.576,N,+515092.943,E
  g_ballsFile.flush()


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
    self.collisionAhead = 10.0,10.0,False,False # far (wide, narrow, override, danger)
    self.lastLeft, self.lastRight = None, None
    self.prevLR = None
    self.poseHistory = []


  def updateExtension( self, robot, id, data ):
    if id == 'remission' and len(data) > 0:
      pass # ignored for FRE 2014

    if id == 'laser' and len(data) > 0:
      self.poseHistory.append( robot.localisation.pose()[:2] )
      if len(self.poseHistory) > 20:
        self.poseHistory = self.poseHistory[-20:]
      step = 10
      data2 = [x == 0 and 10000 or x for x in data]
      arr = [min(i)/1000.0 for i in [itertools.islice(data2, start, start+step) for start in range(0,len(data2),step)]]
      arr.reverse()
      robot.preprocessedLaser = arr

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
      while i <= self.center:
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

      p = robot.localisation.pose()
      if self.center-left >= 0:
        self.lastLeft = combinedPose( (p[0], p[1], p[2]+math.radians(135-5*(self.center-left)) ), (arr[self.center-left],0,0) )
#        viewlog.dumpBeacon(self.lastLeft[:2], color=(0,128,0))
      if self.center+right < len(arr):
        self.lastRight = combinedPose( (p[0], p[1], p[2]+math.radians(135-5*(self.center+right)) ), (arr[self.center+right],0,0) )
#        viewlog.dumpBeacon(self.lastRight[:2], color=(0,128,50))

      if self.verbose:
        if danger and left+right < MIN_GAP_SIZE:
          print "MIN GAP SIZE danger:", left+right
        s = ''
        for i in arr:
          s += (i < 0.5 and 'X' or (i<1.0 and 'x' or (i<1.5 and '.' or ' ')))
        s = "".join( list(s)[:self.center] + ['C'] + list(s)[self.center:] )
        print "'" + s + "'"      
#        print "LRLR\t%d\t%d\t%d" % (left, right, left+right)
#        if self.prevLR :
#          print "LRDIFF", left-self.prevLR[0], right-self.prevLR[1]

      self.line = None # can be defined inside
      if left+right <= MAX_GAP_SIZE: # TODO limit based on minSize, radius and sample angle
        self.center += (right-left)/2
        offset = self.center-len(arr)/2
        self.directionAngle = math.radians( -offset*step/2.0 )
        self.collisionAhead = min(arr[54/3:2*54/3]), min(arr[4*54/9:5*54/9]), (left+right < MIN_GAP_SIZE), danger
        if False: #self.verbose:
          if self.collisionAhead[0] < 0.25:
            print "!!! COLISSION AHEAD !!!", self.collisionAhead
          else:
            print "free space", self.collisionAhead
      elif left < 3 or right < 3:
        # wide row & collision ahead
        if self.verbose:
          print "NEAR", left, right, left+right
        offset = self.center-len(arr)/2
        if left < right:
          offset += 3
        else:
          offset -= 3
        self.directionAngle = math.radians( -offset*step/2.0 )
      else:
        if self.verbose:
          print "OFF", left, right #, left+right, robot.compass
        if False: # hacked, ignoring compass   self.rowHeading:
          self.directionAngle = normalizeAnglePIPI(self.rowHeading-robot.localisation.pose()[2])
          if abs(self.directionAngle) > math.radians(90):
            self.directionAngle = normalizeAnglePIPI(self.rowHeading-robot.localisation.pose()[2]+math.radians(180))
          if self.verbose:
            print "DIFF %.1f" % math.degrees(self.directionAngle)
        else:
          if self.verbose:
            print "PATH GAP"    
          A, B = self.poseHistory[0][:2], self.poseHistory[-1][:2]
          viewlog.dumpBeacon( A, color=(0,0,180) )
          viewlog.dumpBeacon( B, color=(0,30,255) )
          self.line = Line( B, (2*B[0]-A[0], 2*B[1]-A[1]) ) # B+(B-A)
          if self.line and self.line.length < 0.5:
            self.line = None
          else:
            self.center = len(arr)/2
            # reset center
          """if self.prevLR:
            if abs(left-self.prevLR[0]) > 4 and abs(right-self.prevLR[1]) < 3:
              left = self.prevLR[0]
              self.center += (right-left)/2
              offset = self.center-len(arr)/2
              self.directionAngle = math.radians( -offset*step/2.0 )
            elif abs(right-self.prevLR[1]) > 4 and abs(left-self.prevLR[0]) < 3:
              right = self.prevLR[1]
              self.center += (right-left)/2
              offset = self.center-len(arr)/2
              self.directionAngle = math.radians( -offset*step/2.0 )
            else:
              self.directionAngle = 0.0 # if you do not know, go ahead
          else:
            self.directionAngle = 0.0 # if you do not know, go ahead"""
          self.directionAngle = 0.0 # default
          
        if left >= 17 and right >= 17 or left+right >= 40:
          self.endOfRow = True
          if self.verbose and robot.insideField:
            print "laser: END OF ROW"
      self.prevLR = left, right

      pose = robot.localisation.pose()
      goal = combinedPose( (pose[0], pose[1], pose[2]+self.directionAngle), (self.radius, 0, 0) )
      if self.line == None:
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
      if self.verbose and len(data) > 1:
        print data[1]
      cc = [int(x) for x in data[0].split()]
      leftPip = (cc[0] > BALL_SIZE_LIMIT_MIN and cc[0] < BALL_SIZE_LIMIT_MAX)
      rightPip = (cc[1] > BALL_SIZE_LIMIT_MIN and cc[1] < BALL_SIZE_LIMIT_MAX)
      if leftPip or rightPip:
        print "WEED:", leftPip, rightPip
        if leftPip:
          xy = combinedPose(robot.localisation.pose(), (0,0.35,0))[:2]
          reportBall( robot.gpsData )
          viewlog.dumpBeacon( xy, color=(255,255,0) )
        if rightPip:
          xy = combinedPose(robot.localisation.pose(), (0,-0.35,0))[:2]
          reportBall( robot.gpsData )
          viewlog.dumpBeacon( xy, color=(255,255,0) )
        self.lastCamera.append( (self.counter + 0, leftPip, rightPip ) )
      else:
        self.lastCamera.append( (self.counter + 0+4, False, False) )



class FieldRobot:
  rowWidth = 0.75
  rowPotsWidth = 0 #0.45
  def __init__( self, robot, configFilename, verbose = False ):
    self.robot = robot
    self.robot.insideField = True
#    self.robot.fnSpeedLimitController = [self.robot.pauseSpeedFn] 
    self.robot.fnSpeedLimitController = [] 
    self.robot.addExtension( emergencyStopExtension )
#    self.robot.attachGPS()
    self.robot.attachLaser( remission=True )
    self.robot.laser.stopOnExit = False  # for faster boot-up
#    self.robot.attachCamera( cameraExe = "../robotchallenge/rc" ) # TODO what was used?!
#    self.robot.attachCamera( cameraExe = "../robotchallenge/redcone" ) # FRE2015 - task2
    self.robot.attachCamera( cameraExe = "../robotchallenge/dark" ) # FRE2015 - task3
    self.robot.attachHand()
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


  def ver2( self, code, detectWeeds = True, detectBlockedRow = True ):
    print "Field Robot - LASER & CAMERA"

    try:
      # start GPS sooner to get position fix
#      self.robot.gps.start()
      self.robot.laser.start()
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
        self.robot.insideFiled = True
        print "=================  ACTION ", action, "================="
        print "battery:", self.robot.battery
        while not row.endOfRow:
#          sprayer( self.robot, True, True )
          for (speed, angularSpeed) in self.driver.followLineG( row.line ):
#            print "TEST %.2f" % (math.degrees(normalizeAnglePIPI(row.line.angle - self.robot.localisation.pose()[2])))
            if abs(normalizeAnglePIPI(row.line.angle - self.robot.localisation.pose()[2])) > SLOW_DOWN_ANGLE:
              speed *= 0.9
            if self.robot.laserData:
              self.robot.toDisplay = 'OK'
            else:
               self.robot.toDisplay = '--'
               speed, angularSpeed = 0.0, 0.0            
#            if detectBlockedRow and row.collisionAhead[1] < 0.25:

            blockedCamCount = 0
            if detectBlockedRow and self.robot.cameraData:
              camDat, fileName = self.robot.cameraData
              if camDat:
                blockedCamCount = int(camDat.split()[0])
                if blockedCamCount > RED_ALERT:
                  print "CAMERA BLOCKED!!", blockedCamCount

#            print  self.robot.cameraData
#            if detectBlockedRow and row.collisionAhead[2]:
            if detectBlockedRow and row.collisionAhead[3] and blockedCamCount > RED_ALERT:
#            if detectBlockedRow and blocked:
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
        self.robot.insideField = False
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
          self.crossRows( row, math.fabs(action)-1, rowsOnLeft = (action < 0) )
          self.driver.stop()
          if action < 0:
            self.driver.turn( math.radians(90), radius = self.rowWidth/2.0, angularSpeed=math.radians(40) )
#            self.driver.turn( math.radians(90), radius = 0.0, angularSpeed=math.radians(40) )
          else:
            self.driver.turn( math.radians(-90), radius = self.rowWidth/2.0, angularSpeed=math.radians(40) )
#            self.driver.turn( math.radians(-90), radius = 0.0, angularSpeed=math.radians(40) )

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
    self.robot.laser.requestStop()



  def crossRows( self, row, num, rowsOnLeft ):
    "follow N lines without turns"
    dist = num * self.rowWidth
    if num > 1:
      dist += 0.1
    self.driver.goStraight( dist )
    return


  def crossRows0( self, row, num, rowsOnLeft ):
    "follow N lines without turns"
    self.driver.goStraight( num * (self.rowWidth+self.rowPotsWidth)+self.rowPotsWidth )
    return


  def crossRows2( self, row, num, rowsOnLeft ):
    IGNORE_NEIGHBORS = 2
    ROWS_OFFSET = 0.7 #0.5

#    if row.lastLeft:
#      viewlog.dumpBeacon(row.lastLeft[:2], color=(0,128,0))
#    if row.lastRight:
#      viewlog.dumpBeacon(row.lastRight[:2], color=(0,128,50))
    start = self.robot.localisation.pose()[:2]
    viewlog.dumpBeacon( start, color=(255,128,0))

    goal = combinedPose( self.robot.localisation.pose(), (1.0, 0, 0) )    
    line = Line( self.robot.localisation.pose(), goal )
    lastA, lastB = None, None
    ends = []
    while True:
      for cmd in self.driver.followLineG( line ):
        self.robot.setSpeedPxPa( *cmd ) 
        self.robot.update()
        if row.newData:
          row.newData = False
          break
      else:
        print "END OF LINE REACHED!"

      if self.robot.preprocessedLaser != None:
        tmp = self.robot.preprocessedLaser[:]
        tlen = len(tmp)
        if rowsOnLeft:
          tmp = tmp[:tlen/2] + [3.0]*(tlen-tlen/2)
        else:
          tmp = [3.0]*(tlen-tlen/2) + tmp[tlen/2:]
        sarr = sorted([(x,i) for (i,x) in enumerate(tmp)])[:5]
        ax,ai = sarr[0]
        for bx,bi in sarr[1:]:
          if abs(ai-bi) > IGNORE_NEIGHBORS:
            break
        else:
          print "NO 2nd suitable minimum"
        print (ax,ai), (bx,bi)
        if rowsOnLeft:
          offset = -ROWS_OFFSET
          if ai > bi:
            (ax,ai), (bx,bi) = (bx,bi), (ax,ai)
        else:
          offset = ROWS_OFFSET
          if ai < bi: # rows on right
            (ax,ai), (bx,bi) = (bx,bi), (ax,ai)
        p = self.robot.localisation.pose()
        A = combinedPose( (p[0], p[1], p[2]+math.radians(135-5*ai) ), (ax,0,0) )
        B = combinedPose( (p[0], p[1], p[2]+math.radians(135-5*bi) ), (bx,0,0) )
        if lastA == None:
          ends.extend( [A,B] )
          lastA, lastB = A, B
        if self.verbose:
          print "DIST", distance(lastA, A), distance(lastB, B), distance(lastB,A)
        if distance(lastB,A) < 0.2:
          dist = distance(start, self.robot.localisation.pose())
          print "NEXT ROW", dist
          if dist > 0.4:
            ends.append( B ) # new one
          lastA, lastB = A, B
        line = Line(A,B) # going through the ends of rows
        A2 = combinedPose( (A[0], A[1], line.angle), (0, offset, 0) )
        B2 = combinedPose( (B[0], B[1], line.angle), (2.0, offset, 0) )
        line = Line(A2, B2)
        viewlog.dumpBeacon( A[:2], color=(200,0,0) )
        viewlog.dumpBeacon( B[:2], color=(200,128,0) )
        viewlog.dumpBeacon( A2[:2], color=(255,0,0) )
        viewlog.dumpBeacon( B2[:2], color=(255,128,0) )
        if len(ends) > num + 1:
          break
      else:
        print "BACKUP solution!!!"
        goal = combinedPose( self.robot.localisation.pose(), (1.0, 0, 0) )    
        line = Line( self.robot.localisation.pose(), goal )



  def __call__( self ):
    print "RUNNING:", self.configFilename
    if self.configFilename.startswith("cmd:"):
      return eval( self.configFilename[4:] )

#    return self.ver2([-1,1]*10, detectWeeds = False, detectBlockedRow = False)  # Task1
#    return self.ver2( [-3,1,-2,-3,5], detectWeeds = False, detectBlockedRow = True ) # Task2 S-3L-1R-2L-3L-5R-F
    return self.ver2([-2,2,0], detectWeeds = True, detectBlockedRow = False)  # Task3

from eduromaxi import EduroMaxi
import launcher
if __name__ == "__main__": 
  launcher.launch(sys.argv, EduroMaxi, FieldRobot, configFn=setupHandModule)

