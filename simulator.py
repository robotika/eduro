#!/usr/bin/python
"""
  Robot simulator
"""
import math
from ray_trace import rayTrace, combinedPose 

class DummyCAN:
  def sendData(self,id,data):
    pass

class DummyDevice():
  def start( self ):
    pass
  def requestStop( self ):
    pass
 
class SimulatedLaser( DummyDevice ):
  def __init__( self, robot ):
    self.index = 0
    self.data = [2000]*541
#    self.obstacles = [((0.0,-1.8), (5.0,0.0), (1.0, -5.0),(0.0,-1.8))] # TODO load from file
#    self.obstacles = [((-1.0,-0.37), (5.0,-0.37), (5.0, -0.5),(-1.0,-0.5)),
#        ((-1.0,0.37), (5.0,0.37), (5.0, 0.5),(-1.0,0.5)),
#        ((2.0,-0.3), (2.3,-0.3), (2.3,0.3), (2.0,0.3))
#        ] # FRE row
    # FRE 2013
    self.obstacles = []
    w = 0.75
    for i in xrange(-6,6):
      self.obstacles.append( ((-1.0,w*i-0.37), (3.0,w*i-0.37), (3.0, w*i-0.4),(-1.0,w*i-0.4)) )

    self.robot = robot
    self.period = 2 # i.e. for 20Hz, update every self.period cycle

  def updateData( self ):
    pose = (0,0,0)
    if self.robot and self.robot.localisation:
      pose = self.robot.localisation.pose()

    step = 10
    for i in xrange(0,541,step):
      angle = math.radians((i-270)/2.0)
      absPose = combinedPose( pose, (0,0,angle) ) # TODO shift of laser
      simulatedDist = rayTrace( absPose, self.obstacles ) 
      for j in xrange(step):
        if i + j < 541:
          self.data[i+j] = simulatedDist*1000

  def scan( self ):
    self.index += 1
    if self.index % self.period == 0:
      print "scan", self.index
      self.updateData()
      return self.data
    return None


def laserDataExtension( robot, id, data ):
  if id=='laser':
    robot.laserData = data 

class Simulator:
  def __init__( self, maxAcc = 1.0 ):
    self.WHEEL_DISTANCE = 0.245 # small Eduro
    self.WHEEL_DIAMETER = 0.15
    self.maxAcc = maxAcc # could be None as "no limit"
    self.maxAngularAcc = 2.0*maxAcc/self.WHEEL_DISTANCE 
    self.emergencyStop = None
    self.startCableIn = None
    self.switchBlueSelected = None 
    self.tick = 0
    self.lastDistStep = 0.0
    self.lastAngleStep = 0.0 
    self.currentSpeed = 0.0
    self.currentAngularSpeed = 0.0 
    self.time = 0.0 # time in seconds since boot 
    self.sharp = None
    self.localisation = None 
    self.extensions=[]
    self.dataSources=[]
 
    self.can = DummyCAN()
    self.beacons = []
    self.compass = None

  def attachGPS( self ):
    self.gps = DummyDevice()

  def attachLaser( self, remission=False ):
    self.laser = SimulatedLaser( self )
    self.registerDataSource( 'laser', self.laser.scan ) 
    self.laserData = None
    self.laser.pose = None
    self.remissionData = None
    self.addExtension( laserDataExtension ) 

  def attachCamera( self, cameraExe=None ):
    self.camera = DummyDevice()

  def attachHand( self ):
    self.handPosition = None

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


  def setSpeedPxPa( self, speed, angularSpeed ):
    self.currentSpeed = self.currentSpeed*0.9 + 0.05*speed
    self.currentAngularSpeed = self.currentAngularSpeed*0.9 + 0.05*angularSpeed
    self.lastDistStep = self.currentSpeed/20.0
    self.lastAngleStep = self.currentAngularSpeed/20.0
    
  def pauseSpeedFn( self, speedL, speedR ):
    pass

  def update(self):
    self.tick += 1 
    self.time = self.tick/20.0
    if self.tick > 10:
      self.startCableIn = True
      self.switchBlueSelected = True
    if self.tick > 20:
      self.startCableIn = False
    if self.localisation:
      self.localisation.setPxPa( self.lastDistStep, self.lastAngleStep )
    self.beacons = ( (0, -1), (0, -1), (math.radians(45), 0.7), (0,-1) )
    if self.tick > 100:
#      print "Simul colision"
      for (name,e) in self.extensions:
         e( self, 0x185, [0xF8, 0xF8, 0x0, 0xC0] ) # very ugly, problem of simulation extensions not part of the robot
    for (name,e) in self.extensions:
       e( self, 0x80, [] ) 
     
    # send data related to other sources
    for (id,fce) in self.dataSources:
      data = fce()
      if data != None:
        for (name,e) in self.extensions:
          e( self, id, data )
 

   

