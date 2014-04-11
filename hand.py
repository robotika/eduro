#!/usr/bin/env python
"""
  Routines for robotic hand (arm?)
""" 

import math

###########################################################################
from sdoplg import ReadSDO, WriteSDO

HAND_LOWER_LIMIT = 6000 # 8000
HAND_UPPER_LIMIT = 15584
HAND_DESIRED_SPEED = 4000 # i.e. 200 per update
HAND_DESIRED_ACC = 4000 # i.e. get desired speed in 1sec

def handPositionExtension( robot, id, data ):
  if id == 0x28C: 
    assert( len(data) == 8 )
    robot.handPosition = data[1]*256 + data[0]


def setupHandModule( can ):
  reader = ReadSDO( 12, 0x1801, 5 )
  for packet in reader.generator():
    if packet != None:
      can.sendData( *packet )
    reader.update( can.readPacket() )
  print "RESUT DATA (before):", reader.result

  #  writer = WriteSDO( 12, 0x1801, 2, [1] )
  writer = WriteSDO( 12, 0x1801, 5, [50, 0] )
  for cmd in writer.generator():
    if cmd:
      can.sendData( *cmd )
    writer.update( can.readPacket() )

  for packet in reader.generator():
    if packet != None:
      can.sendData( *packet )
    reader.update( can.readPacket() )
  print "RESUT DATA (after):", reader.result

  reader = ReadSDO( 12, 0x2100, 1 )
  for packet in reader.generator():
    if packet != None:
      can.sendData( *packet )
    reader.update( can.readPacket() )
  print "Function Select, 8bit channel status:", reader.result

  """
# set servo on 3rd channel
  writer = WriteSDO( 12, 0x2100, 1, [0x04] )
  for cmd in writer.generator():
    if cmd:
      can.sendData( *cmd )
    writer.update( can.readPacket() )

  reader = ReadSDO( 12, 0x2100, 1 )
  for packet in reader.generator():
    if packet != None:
      can.sendData( *packet )
    reader.update( can.readPacket() )
  print "Function Select, 8bit channel status AFTER:", reader.result
  """


def handServo( can, value ):
  can.sendData( 0x30C, [0,0,0,0, value & 0xFF, (value >> 8)&0xFF, 0,0] ) 


def handUpG( robot, timeout=2.0, infinite=False, defaultServoValue=-5000 ):
  kP, kI = 1.0/20.0, 0.2/20.0 # PI regulator constant
  ret = False
  print "Hand UP"
  startTime = robot.time
  handServo( robot.can, defaultServoValue)
  prev = None
  errSum = 0
  while True:
    yield
    time = robot.time - startTime
    desiredSpeed = min(time * HAND_DESIRED_ACC, HAND_DESIRED_SPEED) # acc=1
    if robot.handPosition != None and robot.handPosition + desiredSpeed > HAND_UPPER_LIMIT:
      time = math.sqrt( math.fabs( HAND_UPPER_LIMIT - robot.handPosition )/(HAND_DESIRED_ACC*2.0) )
      desiredSpeed = max( HAND_DESIRED_SPEED/4, time*HAND_DESIRED_SPEED ) # slow down near final position
    print robot.handPosition
    if prev and robot.handPosition:
      speed = 20*(robot.handPosition-prev)
      err = desiredSpeed - speed
      errSum += err
#      print "ERR", err, errSum, (robot.handPosition, speed, int(desiredSpeed))
      val = defaultServoValue - kP * err - kI * errSum
      if val < -10000:
        val = -10000
      handServo( robot.can, int(val) )

    prev = robot.handPosition
    if robot.handPosition != None and robot.handPosition > HAND_UPPER_LIMIT:
      ret = True
      break
    if timeout != None:
      if robot.time > startTime + timeout:
        break
  handServo( robot.can, 0 )
  for i in xrange(20):
    yield
  if infinite:
    while 1:
      yield # hmm, revise ...

def handUp( robot, timeout=2.0 ):
  for x in handUpG( robot, timeout ):
    robot.update()


def handDownG( robot, timeout=1.0, infinite=False, defaultServoValue=8000, limit=HAND_LOWER_LIMIT ):
  kP, kI = 1.0/20.0, 0.2/20.0 # PI regulator constant
  ret = False
  print "Hand DOWN"
  startTime = robot.time
  handServo( robot.can, defaultServoValue)
  prev = None
  errSum = 0
  while True:
    yield
    time = robot.time - startTime
    desiredSpeed = -min(time * HAND_DESIRED_ACC, HAND_DESIRED_SPEED) # acc=1
    if robot.handPosition != None and robot.handPosition + desiredSpeed < limit:
      time = math.sqrt( math.fabs( limit - robot.handPosition )/(HAND_DESIRED_ACC*2.0) )
      desiredSpeed = -max( HAND_DESIRED_SPEED/4, time*HAND_DESIRED_SPEED ) # slow down near final position
    print robot.handPosition
    if prev and robot.handPosition:
      speed = 20*(robot.handPosition-prev)
      err = desiredSpeed - speed
      errSum += err
#      print "ERR", err, errSum, (robot.handPosition, speed, int(desiredSpeed))
      val = defaultServoValue - kP * err - kI * errSum
      if val > 10000:
        val = 10000
      handServo( robot.can, int(val) )
    prev = robot.handPosition
    if robot.handPosition != None and robot.handPosition < limit:
      ret = True
      break
    if timeout != None:
      if robot.time > startTime + timeout:
        break
  handServo( robot.can, 0 )
  for i in xrange(20): # servo Neutral
    yield
  if infinite:
    while 1:
      yield # hmm, revise ...

def handDown( robot, timeout=1.0 ):
  for x in handDownG( robot, timeout ):
    robot.update()

