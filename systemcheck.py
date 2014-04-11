#!/usr/bin/env python

import math

from driver import Driver, normalizeAnglePIPI
from eduro import EmergencyStopException
from localisation import SimpleOdometry

class SystemCheck:
  '''
  Checks whether the robot works.
  '''
  def __init__(self, robot, configFile, verbose=False):
    self.robot = robot
    self.configFile = configFile
    self.verbose = verbose

    self.robot.attachGPS()
    self.robot.attachLaser()
    self.robot.attachCamera(cameraExe='./king')
    #self.robot.attachRemoteCamera(('10.0.0.11', 8431))
    #self.robot.attachRemoteCamera(('10.0.0.17', 8431))
    self.robot.attachSonar()
    self.robot.attachEmergencyStopButton()

    self.driver = Driver( self.robot,
                          maxSpeed = 0.3,
                          maxAngularSpeed = math.radians(60))

    if self.robot.pauseSpeedFn not in self.robot.fnSpeedLimitController:
      self.robot.fnSpeedLimitController.append(self.robot.pauseSpeedFn)

  def __call__(self):
    try:
      # start GPS sooner to get position fix
      self.robot.gps.start()
      if getattr( self.robot.laser, 'startLaser', None ):
        # trigger rotation of the laser, low level function, ignore for log files
        print "Powering laser ON"
        self.robot.laser.startLaser() 
      self.robot.waitForStart()
      self.robot.laser.start()
      self.robot.camera.start()
      self.robot.localisation = SimpleOdometry()

      self.check()
    except EmergencyStopException, e:
      print "EmergencyStopException"

    self.robot.gps.requestStop()
    self.robot.laser.requestStop()
    self.robot.camera.requestStop()

  def check(self):
    driver = self.driver # for less typing

    STRAIGHT_DISTANCE = 0.5 # [m]
    TURN_BACK = math.pi # [rad]
    STRAIGHT_TIMEOUT = 8 * STRAIGHT_DISTANCE / driver.maxSpeed # [s]
    TURN_TIMEOUT = 8 * TURN_BACK / driver.maxAngularSpeed # [s]

    [X, Y, PHI] = range(3)

    # Move around a bit
    startHeading = math.radians(self.robot.compass * 0.1)
    self.__performMove(driver.goStraightG(STRAIGHT_DISTANCE), STRAIGHT_TIMEOUT)
    self.__performMove(driver.turnG(TURN_BACK), TURN_TIMEOUT)
    pose = self.robot.localisation.pose()
    traveledDist = math.hypot(pose[X], pose[Y])
    self.__performMove(driver.goStraightG(STRAIGHT_DISTANCE), STRAIGHT_TIMEOUT)
    backHeading = math.radians(self.robot.compass * 0.1)
    self.__performMove(driver.turnG(-TURN_BACK), TURN_TIMEOUT)

    def report(sensorName, status):
      if status == True:
        msg = 'ok'
      elif status == False:
        msg = 'failed'
      else:
        msg = status
      print '%s%s%s' % (sensorName, ' ' * max(1, 12 - len(sensorName)), msg)

    def moveCheck():
      return traveledDist > 0.5 * STRAIGHT_DISTANCE

    def laserCheck():
      return bool(self.robot.laserData) and any(x > 0 for x in self.robot.laserData)

    def cameraCheck():
      return self.robot.cameraData and self.robot.cameraData[0] is not None

    def gpsCheck():
      return self.robot.gpsData and self.robot.gpsData[0] is not None

    def compassCheck():
      return (self.robot.compass is not None
              and abs(normalizeAnglePIPI(backHeading - startHeading)) > math.pi / 2)

    def sonarCheck():
      return bool(self.robot.sonar)

    # Check the sensor information now
    report('Move:', moveCheck())
    report('Laser:', laserCheck())
    report('Camera:', cameraCheck())
    report('GPS:', gpsCheck())
    report('Compass:', compassCheck())
    report('Sonar:', sonarCheck())

  def __performMove(self, strategy, timeout):
    startTime = self.robot.time

    for cmd in strategy:
      self.robot.setSpeedPxPa(*cmd)
      self.robot.update()
      if self.robot.time >= startTime + timeout:
        break

if __name__ == '__main__':
  import sys

  import eduromaxi
  import launcher

  launcher.launch(sys.argv, eduromaxi.EduroMaxi, SystemCheck)

