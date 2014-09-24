#!/usr/bin/python
"""
  EduroMaxi - first outdoor robot of Eduro series
  3-wheel configuration:
     width = 0.39m
     length = 0.62m
     height = 0.54m (with camera support)
"""

from robot import Robot

# for building the robot
from robot import SourceLogger, SIDE_LEFT, SIDE_RIGHT
from gps import GPS, DummyGPS, timeName
from camera import Camera, ImageProc, RemoteCamera, timeName, DummyProc, DummyCamera
from laser import LaserUSB, LaserIP
from eduro import emergencyStopExtension
from os import path, sep
from hand import handPositionExtension
from rfid import RFID
from barcode import BarcodeReader

def processedRFID( rfidData ):
  if rfidData and len(rfidData) == 10:
    if rfidData.startswith("B1 "):
      try:
        return int(rfidData.split()[1]),int(rfidData.split()[2]) # index and intensity
      except:
        print "RFID parse error", rfidData
  return None


class EduroMaxi( Robot ):
  def __init__( self, can=None, maxAcc = 0.5, replyLog=None, metaLog=None):
    Robot.__init__( self, can, maxAcc, replyLog, metaLog=metaLog ) 
    self.WHEEL_DISTANCE = 0.315
    #wd = 0.26/4.0 # with gear  1:4
    wd = 427.0 / 445.0 * 0.26/4.0 # with gear  1:4
    delta = 0.00015
    self._WHEEL_DIAMETER = {SIDE_LEFT: wd+delta, SIDE_RIGHT: wd-delta} #TODO: different wheel diameters
    self.fnSpeedLimitController = []

  def metaLogName( self, prefix ):
    for line in self.metaLog:
      if line.startswith( prefix ):
        return path.dirname(self.replyLog)+sep+line.split()[1].split("/")[-1]
    return None

  def attachGPS(self):
    if self.replyLog is None:
      self.gps = GPS( verbose=0 )
      name = timeName( "logs/src_gps_", "log" )
      if self.metaLog:
        self.metaLog.write("GPSLOG:\t" + name + "\n")
        self.metaLog.flush()
      self.registerDataSource( 'gps', SourceLogger( self.gps.coord, name ).get )
    else:
      self.gps = DummyGPS()
      if self.metaLog:
        gpsSrcLog = self.metaLogName( "GPSLOG:" )
      else:
        gpsSrcLog = self.replyLog[:-18]+"src_gps_"+self.replyLog[-17:]
      print "GPSLOG:", gpsSrcLog
      self.registerDataSource( 'gps', SourceLogger( None, gpsSrcLog ).get )
    self.gpsData = None
    self.addExtension( gpsDataExtension )


  def attachBarcodeReader(self):
    if self.replyLog is None:
      self.barcode = BarcodeReader()
      name = timeName( "logs/src_barcode_", "log" )
      if self.metaLog:
        self.metaLog.write("BARCODELOG:\t" + name + "\n")
        self.metaLog.flush()
      self.registerDataSource( 'barcode', SourceLogger( self.barcode.getCode, name ).get )
    else:
      self.barcode = DummyGPS()
      if self.metaLog:
        barSrcLog = self.metaLogName( "BARCODELOG:" )
      else:
        barSrcLog = self.replyLog[:-18]+"src_barcode_"+self.replyLog[-17:]
      print "BARCODELOG:", barSrcLog
      self.registerDataSource( 'barcode', SourceLogger( None, barSrcLog ).get )
    self.barcodeData = None
    self.addExtension( barcodeDataExtension )


  def attachCamera(self, cameraExe=None, url="http://192.168.0.99/img.jpg", sleep=None):
    if self.replyLog is None:
      if cameraExe:
        proc = ImageProc(exe=cameraExe, verbose=0, priority=10)
      else:
        proc = DummyProc( sleep=sleep )
      self.camera = Camera( proc,
                            verbose=0,
                            url=url,
                            sleep=sleep )
      name = timeName( "logs/src_camera_", "log" )
      if self.metaLog:
        self.metaLog.write("CAMERALOG:\t" + name + "\n")
        self.metaLog.flush()
      self.registerDataSource( 'camera', SourceLogger( self.camera.lastResultEx, name ).get )
    else:
      self.camera = DummyCamera()
      if self.metaLog:
        cameraSrcLog = self.metaLogName( "CAMERALOG:" )
      else:
        cameraSrcLog = self.replyLog[:-18]+"src_camera_"+self.replyLog[-17:]
      print "CAMERALOG:", cameraSrcLog
      self.registerDataSource( 'camera', SourceLogger( None, cameraSrcLog ).get )
    self.cameraData = None
    self.addExtension( cameraDataExtension )

  def attachEmergencyStopButton(self):
    self.addExtension( emergencyStopExtension )

  def attachLaser( self, remission=False, usb=False, pose=None, index=None, errLog=None ):
    if index == None:
      strIndex = ''
    else:
      strIndex = str(index)

    if self.replyLog is None:
      if usb:
        laser = LaserUSB( remission=remission, errLog=errLog )
      else:
        laser = LaserIP( remission=remission )
      name = timeName( "logs/src_laser"+strIndex+"_", "log" )
      if self.metaLog:
        self.metaLog.write("LASERLOG:\t" + name + "\n")
        self.metaLog.flush()
      self.registerDataSource( 'laser'+strIndex, SourceLogger( laser.scan, name ).get )
      if remission:
        name = timeName( "logs/src_remission"+strIndex+"_", "log" )
        if self.metaLog:
          self.metaLog.write("REMISSIONLOG:\t" + name + "\n")
          self.metaLog.flush()
        self.registerDataSource( 'remission'+strIndex, SourceLogger( laser.remission, name ).get )
    else:
      laser = DummyGPS() # just start
      if self.metaLog:
        laserSrcLog = self.metaLogName( "LASERLOG:" )
      else:
        laserSrcLog = self.replyLog[:-18]+"src_laser"+strIndex+"_"+self.replyLog[-17:]
      print "LASERLOG:", laserSrcLog
      self.registerDataSource( 'laser'+strIndex, SourceLogger( None, laserSrcLog ).get )
      if remission:
        if self.metaLog:
          remissionSrcLog = self.metaLogName( "REMISSIONLOG:" )
        else:
          remissionSrcLog = self.replyLog[:-18]+"src_remission"+strIndex+"_"+self.replyLog[-17:]
        print "REMISSIONLOG:", remissionSrcLog
        self.registerDataSource( 'remission'+strIndex, SourceLogger( None, remissionSrcLog ).get )
    if index == None:
      self.laser = laser
      self.laserData = None
      self.remissionData = None
      self.laser.pose = pose
      self.addExtension( laserDataExtension )
    else:
      assert( index == 2 )
      self.laser2 = laser
      self.laserData2 = None
      self.remissionData2 = None
      self.laser2.pose = pose
      self.addExtension( laserDataExtension2 )

  def attachRemoteCamera(self, address=('localhost', 8431)):
    if self.replyLog is None:
      self.camera = RemoteCamera( address, verbose=0 )
      self.registerDataSource( 'camera', SourceLogger( self.camera.lastResultEx, timeName( "logs/src_camera_", "log" ) ).get )
    else:
      self.camera = DummyGPS() # TODO some dummy device with start(), requestStop() functions
      cameraSrcLog = self.replyLog[:-18]+"src_camera_"+self.replyLog[-17:]
      print "CAMERALOG:", cameraSrcLog
      self.registerDataSource( 'camera', SourceLogger( None, cameraSrcLog ).get )
    self.cameraData = None
    self.addExtension( cameraDataExtension )

  def attachSonar(self):
    # The sonar data come from the CAN/CAN log, not prom a stand-alone file.
    self.sonar = None
    self.addExtension(sonarExtension)

  def attachHand(self):
    # The hand is attached via CAN bus
    self.handPosition = None
    self.addExtension( handPositionExtension )

  def attachRFID(self):
    if self.replyLog is None:
      self.rfid = RFID()
      name = timeName( "logs/src_rfid_", "log" )
      if self.metaLog:
        self.metaLog.write("RFIDLOG:\t" + name + "\n")
        self.metaLog.flush()
      self.registerDataSource( 'rfid', SourceLogger( self.rfid.data, name ).get )
    else:
      self.rfid = DummyGPS() # just start
      if self.metaLog:
        rfidSrcLog = self.metaLogName( "RFIDLOG:" )
      else:
        rfidSrcLog = self.replyLog[:-18]+"src_rfid_"+self.replyLog[-17:]
      print "RFIDLOG:", rfidSrcLog
      self.registerDataSource( 'rfid', SourceLogger( None, rfidSrcLog ).get )
    self.rfidData = None
    self.rfidDataRaw = None
    self.addExtension( rfidDataExtension )

  def attachProcessingNode( self ):
    if self.replyLog is None:
      self.node = SocketExtension()
      name = timeName( "logs/src_node_", "log" )
      if self.metaLog:
        self.metaLog.write("NODELOG:\t" + name + "\n")
        self.metaLog.flush()
      self.registerDataSource( 'node', SourceLogger( self.node.data, name ).get )
    else:
      self.node = DummyGPS() # just start
      if self.metaLog:
        nodeSrcLog = self.metaLogName( "NODELOG:" )
      else:
        nodeSrcLog = self.replyLog[:-18]+"src_node_"+self.replyLog[-17:]
      print "NODELOG:", nodeSrcLog
      self.registerDataSource( 'node', SourceLogger( None, nodeSrcLog ).get )
    self.nodeData = None
    self.addExtension( nodeDataExtension )


def gpsDataExtension( robot, id, data ):
  if id=='gps':
    robot.gpsData = data

    if robot.localisation is not None:
      robot.localisation.updateGPS(data)

def barcodeDataExtension( robot, id, data ):
  if id=='barcode':
    robot.barcodeData = data

def cameraDataExtension( robot, id, data ):
  if id=='camera':
    robot.cameraData = data

def laserDataExtension( robot, id, data ):
  if id=='laser':
    robot.laserData = data
  if id=='remission':
    robot.remissionData = data

def laserDataExtension2( robot, id, data ):
  if id=='laser2':
    robot.laserData2 = data
  if id=='remission2':
    robot.remissionData2 = data

def sonarExtension( robot, id, data ):
  if id == 0x186: 
    assert( len(data) )
    robot.sonar = (data[1]*256 + data[0])*340.0/2.0/1000000.0

def trackerDataExtension( robot, id, data ):
  if id=='tracker':
    robot.trackerData = data

def rfidDataExtension( robot, id, data ):
  if id=='rfid':
    robot.rfidDataRaw = data
    robot.rfidData = processedRFID( data )

def nodeDataExtension( robot, id, data ):
  if id=='node':
    robot.nodeData = data

def buildRobot( robot, attachedGPS = False, attachedCamera = False, cameraExe = None, attachedLaser = False, replyLog = None ):
  "attach various sensors to 'robot base'"
  robot.replyLog = replyLog
  if attachedGPS:
    robot.attachGPS()
 
  if attachedCamera:
    robot.attachCamera(cameraExe)

  if attachedLaser:
    robot.attachLaser()

  return robot

