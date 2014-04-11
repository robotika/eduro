#!/usr/bin/python
"""
  Abstract robot interface over serial port.
    ./robot.py [<log file> [<additional info>]]
"""

from can import CAN
import ctypes
import time
import math
import localisation

from compass import analyseCompass

SIDE_LEFT = 0
SIDE_RIGHT = 1

ACC_SMOOTHING = 0.1 # scale factor (0..1) for old measurement, 0.0 = only new measurements will be used

def angleDeg( degrees ):
  return math.pi * degrees / 180.0

def sint16( data ):
  ret = data[1]*256+data[0]
  if ret > 0x8000:
    ret = ret-0x10000
  return ret

class SourceLogger:
  def __init__( self, sourceGet, filename ):
    self.sourceGet = sourceGet
    self.counter = 0
    self.prevData = None
    if self.sourceGet != None:
      self.file = open( filename, 'w' )
    else:
      self.file = open( filename )
      try:
        self.counterLimit = int(self.file.readline())
      except ValueError:
        # case when given device was not started
        print "EMPTY FILE!!!"
        self.counterLimit = 10000 # "infinity"


  def get( self ):
    if self.sourceGet != None:
      data = self.sourceGet()
      if data != None and data != self.prevData:
        self.file.write( str(self.counter) + "\n" )
        self.file.write( repr(data) + "\n" )
        self.file.flush()
        self.counter = 1
        self.prevData = data
        return self.prevData
    else:
      if self.counter >= self.counterLimit:
        self.counter = 1
        self.prevData = eval( self.file.readline() )
        nextCnt = self.file.readline()
        self.counterLimit = float('inf') if nextCnt == '' else int(nextCnt)
        return self.prevData
    self.counter += 1
    return None

  def generator( self ):
    assert( self.sourceGet is None )
    while 1:
      line = self.file.readline()
      if len(line) == 0:
        break
      self.prevData = eval( line )
      self.counterLimit = int(self.file.readline())
      yield self.prevData

  def __del__( self ):
    if self.sourceGet != None:
      self.file.write( str(self.counter) + "\n" )
    self.file.close()


class Robot():
  
  UPDATE_TIME_FREQUENCY = 20.0  # Hz

  def __init__( self, can=None, maxAcc = 1.0, replyLog = None, metaLog = None ):
    self.metaLog = metaLog
    self.replyLog = replyLog
    if not can:
      can=CAN()
      can.resetModules()
    print can
    self.can = can
    self.WHEEL_DISTANCE = 0.245 # small Eduro
    self._WHEEL_DIAMETER = 0.15
    self.maxAcc = maxAcc # could be None as "no limit"
    self.maxAngularAcc = 2.0*maxAcc/self.WHEEL_DISTANCE
    self.swappedMotors = False

    self.SpeedL,self.SpeedR=0,0
    self.prevEncL = None
    self.prevEncR = None
    self.encData = {}
    self._rampLastLeft, self._rampLastRight = 0.0, 0.0
    self.lastDistStep = 0.0
    self.lastAngleStep = 0.0
    self.currentSpeed = 0.0
    self.currentAngularSpeed = 0.0
    self.time = 0.0 # time in seconds since boot
    self.battery = None
    
    self.emergencyStop = None
    self.startCableIn = None
    self.switchBlueSelected = None
    self.buttonPause = None
    self.cmdLEDBlue = 0xFF
    self.cmdLEDRed  = 0xFF
    self.toDisplay = None
    self.beep = None
    self.compass = None
    self.compassRaw = None
    self.compassAcc = None
    self.compassAccRaw = None
    self.sharp = None

    self.fnSpeedLimitController = [self.sharpSpeedFn, self.pauseSpeedFn]
    self.localisation = None
    self.extensions=[]
    self.dataSources=[]

    self.modulesForRestart = []
    self.can.sendOperationMode()
  
  def __del__(self):
    self.can.sendPreoperationMode()

  def addExtension(self, function, name =""):
    self.extensions.append( (name, function) )
  
  def removeExtension(self, name):
    newExt = []
    for (n, f) in self.extensions:
      if name != n:
        newExt.append( (n,f) )
    self.extensions = newExt


  def registerDataSource( self, name, function ):
    self.dataSources.append( (name, function) )

  def unregisterDataSource( self, name ):
    newSrc = []
    for (n, f) in self.dataSources:
      if name != n:
        newSrc.append( (n,f) )
    self.dataSources = newSrc


  def wheelDiameter(self, side=None):
    if hasattr(self._WHEEL_DIAMETER, "__getitem__"):
      if side is None:
        return (self.wheelDiameterRight() + self.wheelDiameterLeft()) / 2.0
      else:
        return self._WHEEL_DIAMETER[side]
    else:
      return self._WHEEL_DIAMETER

  def wheelDiameterLeft(self):
    return self.wheelDiameter(SIDE_LEFT)

  def wheelDiameterRight(self):
    return self.wheelDiameter(SIDE_RIGHT)

  def diffEnc( self, a, b ):
    ret = a - b
    if ret > 0x80000000:
      return ret - 0x100000000
    if ret < -0x80000000:
      return ret + 0x100000000
    return ret


  def updateEnc( self, id, data ):
    if id == 0x181 or id == 0x182:
      assert( len(data)>=4 )
      motorId = id & 0x0F
      if motorId not in self.modulesForRestart:
        e = data[0]|(data[1]<<8)|(data[2]<<16)|(data[3]<<24)
        self.encData[id] = e

    if id == 0x80: # SYNC
      if 0x181 in self.encData and 0x182 in self.encData:
        distL, distR = 0, 0
        if self.prevEncL != None:
          distL = self.diffEnc( self.encData[ 0x181 ], self.prevEncL )
        self.prevEncL = self.encData[ 0x181 ]

        if self.prevEncR != None:
          distR = self.diffEnc( self.encData[ 0x182 ], self.prevEncR )
        self.prevEncR = self.encData[ 0x182 ]

#        print 'E', distL, distR

        scaleR = self.wheelDiameterRight() * math.pi/0x10000
        scaleL = self.wheelDiameterLeft() * math.pi/0x10000

        metricL = scaleL * distL
        metricR = scaleR * distR

        dist = (metricL + metricR)/2.0
        angle = (metricR - metricL)/self.WHEEL_DISTANCE
        if self.swappedMotors:
          dist = -dist
        if self.localisation:
          self.localisation.setPxPa( dist, angle )
        self.lastDistStep = dist
        self.lastAngleStep = angle
        self.currentSpeed = dist * self.UPDATE_TIME_FREQUENCY
        self.currentAngularSpeed = angle * self.UPDATE_TIME_FREQUENCY

      self.encData = {}

  def updateEmergencyStop( self, id, data ):
    if id == 0x8A:
      print data
      assert( len(data) == 8 )
      self.emergencyStop = (data[0:2] == [0,0x10]) and (data [3:] == [0,0,0,0,0])
      print "EmergencyStop:", self.emergencyStop
    
  def updateCompass( self, id, data ):
#    if (id & 0xF) == 0x7:
#      print hex(id), data
    if id == 0x187:
      pass # used processed 3D compass instead
      #assert( len(data) == 2 ) #Note: Not true with the 3D compass
      #self.compass = data[0] + 256*data[1]

      #self.pitch = ctypes.c_int16(data[2] + 256*data[3]).value / 10.0
      #self.roll = ctypes.c_int16(data[4] + 256*data[4]).value / 10.0
      #print 'compass:', self.compass / 10.0
      #print 'pitch:', self.pitch
      #print 'roll:', self.roll

      #if self.localisation:
      #  self.localisation.updateCompass( self.compass )
    if id == 0x387: # 3D accelerometer
      self.compassAccRaw = (sint16(data[0:2]), sint16(data[2:4]), sint16(data[4:6]) )
      if self.compassAcc != None:
        self.compassAcc = (
            (1.0-ACC_SMOOTHING)*self.compassAcc[0]+ACC_SMOOTHING*self.compassAccRaw[0], 
            (1.0-ACC_SMOOTHING)*self.compassAcc[1]+ACC_SMOOTHING*self.compassAccRaw[1], 
            (1.0-ACC_SMOOTHING)*self.compassAcc[2]+ACC_SMOOTHING*self.compassAccRaw[2])
      else:
        self.compassAcc = self.compassAccRaw # init value for averaging
    if id == 0x487: # 3D raw compass data
      self.compassRaw = (sint16(data[0:2]), sint16(data[2:4]), sint16(data[4:6]) )
      if self.compassAcc:
        self.compass = analyseCompass( self.compassRaw, self.compassAcc )
        if self.localisation:
          self.localisation.updateCompass( self.compass )

  def updateButtons( self, id, data ):
    if id == 0x28A:
      assert( len(data) == 2 )
      mask = data[1]*256 + data[0]
      self.startCableIn       = (mask & 0x0100 ) != 0
      self.switchBlueSelected = (mask & 0x0200 ) == 0
      self.buttonPause = not self.switchBlueSelected # paused iff red LED is on
      if self.switchBlueSelected:
        self.cmdLEDBlue = 1
        self.cmdLEDRed = 0
      else:
        self.cmdLEDBlue = 0
        self.cmdLEDRed = 1

  def updateSharps( self, id, data ):
    if id == 0x183:
      assert( len(data) == 8 )
      s0 = data[0] + 256*data[1]
      s1 = data[2] + 256*data[3]
      s2 = data[4] + 256*data[5]
      s3 = data[6] + 256*data[7]
      self.sharp = (s0, s1, s2, s3)

  def updateBattery( self, id, data ):
    if id == 0x18B:
      self.battery = (data[1]*256+data[0])/100.0

  def checkAndRestartModules( self, id, data ):
    # test if all modules are in Operation mode, if not restart them
    if id & 0xFF0 == 0x700: # heart beat message
      moduleId = id & 0xF
      assert( len(data) == 1 )
      if data[0] != 5:
        self.can.printPacket( id, data )
        if not moduleId in self.modulesForRestart:
          self.modulesForRestart.append( moduleId )
          print "RESET", moduleId
          self.can.sendData( 0, [129,moduleId] )
          if moduleId in [0x01, 0x02]:
            if (0x180 | moduleId) in self.encData:
              # The encoder information is invalid during a reset
              del self.encData[0x180 | moduleId]
            if moduleId == 0x01:
              self.prevEncL = None
            if moduleId == 0x02:
              self.prevEncR = None
        elif data[0] == 127: # restarted and in preoperation
          print "SWITCH TO OPERATION", moduleId
          self.can.sendData( 0, [1,moduleId] ) 
      elif moduleId in self.modulesForRestart:
        print "RUNNING", moduleId
        self.modulesForRestart.remove( moduleId )

  def setSpeedPxPa( self, speed, angularSpeed ):
    self.SpeedL = speed - angularSpeed * self.WHEEL_DISTANCE / 2.0
    self.SpeedR = speed + angularSpeed * self.WHEEL_DISTANCE / 2.0

  def sendLEDs( self ):
    self.can.sendData( 0x30A, [0xFF, 0xFF, self.cmdLEDBlue, self.cmdLEDRed ] )

  def sendDisplay( self ):
    if self.toDisplay:
      self.can.sendData( 0x209, [ord(x) for x in self.toDisplay[:2]] )

  def sendBeep( self ):
    if self.beep != None:
      self.can.sendData( 0x309, [self.beep] )

  def sendSpeed( self ):
    if any(motorId in self.modulesForRestart for motorId in [0x01, 0x02]):
      # There is a motor in reset => we must stop (even aggressively)
      left = right = 0
    else:
      scaleR = 1000.0 / ( math.pi * self.wheelDiameterRight() )
      scaleL = 1000.0 / ( math.pi * self.wheelDiameterLeft() )
      tmpSpeedL = self.SpeedL
      tmpSpeedR = self.SpeedR
      for fn in self.fnSpeedLimitController:
        (tmpSpeedL, tmpSpeedR ) = fn( tmpSpeedL, tmpSpeedR )
      if self.maxAcc:
        # use ramps
        maxSpeedStep = self.maxAcc / self.UPDATE_TIME_FREQUENCY;
        if math.fabs( tmpSpeedL - self._rampLastLeft ) > maxSpeedStep or \
           math.fabs( tmpSpeedR - self._rampLastRight ) > maxSpeedStep:
          frac = maxSpeedStep / max( math.fabs( tmpSpeedL - self._rampLastLeft ), math.fabs( tmpSpeedR - self._rampLastRight ) )
          tmpSpeedL = self._rampLastLeft + frac * ( tmpSpeedL - self._rampLastLeft )
          tmpSpeedR = self._rampLastRight + frac * ( tmpSpeedR - self._rampLastRight )
          self._rampLastLeft = tmpSpeedL
          self._rampLastRight = tmpSpeedR

      left = int(scaleL * tmpSpeedL)
      right = int(scaleR * tmpSpeedR)
      if self.swappedMotors:
        tmp = left
        left = -right
        right = -tmp

  #    print 'S', left, right
      maxLim = 4000
      if left > maxLim:
        right = right*maxLim/left
        left = maxLim
      if left < -maxLim:
        right = -right*maxLim/left
        left = -maxLim
      if right > maxLim:
        left = left*maxLim/right
        right = maxLim
      if right < -maxLim:
        left = -left*maxLim/right
        right = -maxLim

    self.can.sendData(0x201,[left&0xff,(left>>8)&0xff,right&0xff,(right>>8)&0xff])

  def sharpSpeedFn( self, speedL, speedR ):
    if self.sharp and (speedL + speedR) > 0: # going forward
      for s in self.sharp:
        if s > 10000:
          return (0,0)
      for s in self.sharp:
        if s > 8000:
          return (speedL/2, speedR/2)
    return speedL, speedR

  def pauseSpeedFn( self, speedL, speedR ):
    if self.buttonPause:
      return 0.0, 0.0
    return speedL, speedR

  def update(self):
    while 1:
      id,data=self.can.readPacket()
      self.updateEnc(id,data)
      self.updateEmergencyStop( id, data )
      self.updateCompass(id,data)
      self.updateButtons(id,data)
      self.updateSharps( id,data )
      self.updateBattery( id, data )
      self.checkAndRestartModules( id, data )
      for (name,e) in self.extensions:
        e( self, id, data )
      if id == 0x80: # make sure that all updates get also termination SYNC (0x80)
        break

    # send data related to other sources
    for (id,fce) in self.dataSources:
      data = fce()
      if data != None:
        for (name,e) in self.extensions:
          e( self, id, data )

    self.time += 1.0/self.UPDATE_TIME_FREQUENCY  
    self.sendSpeed()
    self.sendLEDs()
    self.sendDisplay()
    self.sendBeep()

  def waitForStart(self):
    print "Waiting for start cable insertion ..."
    while self.startCableIn is None or not self.startCableIn:
      self.setSpeedPxPa( 0.0, 0.0 )
      self.update()
    print "READY & waiting for start ..."
    while self.startCableIn is None or self.startCableIn:
      self.setSpeedPxPa( 0.0, 0.0 )
      self.update()
    print "!!! GO !!!" 


def main( argv ):
  from can import ReplyLog, DummyMemoryLog, ReplyLogInputsOnly

  if len( argv ) < 2:
    r=Robot()
  elif len( argv ) == 2:
    r=Robot( CAN(ReplyLog( argv[1] ), skipInit = True ) )
  else:
    can = CAN( com = DummyMemoryLog() ) # skip CAN initialization
    can.com = ReplyLogInputsOnly( argv[1] ) # switch to log file
    r = Robot( can )
#    r.localisation = localisation.SimpleOdometry()
    r.localisation = localisation.KalmanFilter()
    print r.localisation
#    try:
#      while 1:
    for i in range(10000):
      r.update()
      print r.compass
#      print r.prevEncL
    #except : #ignore termination
    #  pass
    print r.localisation.pose()
    return

  s = "AHOJ jak se mas? Tototo je prvni dlouhy text. ".upper() * 2
  r.beep = 1
  for i in range( len(s)-2 ):
    r.toDisplay = s[i:i+2]
    for j in range(7):
      r.update()
  r.beep = 0
  r.update()
  del r
  return

  # wait for start cable to be removed
  while r.startCableIn is None or r.startCableIn == True:
    r.update()
  
  r.setSpeedPxPa( 0.1, angleDeg(0) )
  r.update()
  r.update()
  r.update()
  assert( r.compass != None )

  for i in range(60):
    r.update()
#    print "Compass", r.compass
    print "Sharps", r.sharp

  del r


if __name__ == "__main__":
  import sys
  main( sys.argv )

