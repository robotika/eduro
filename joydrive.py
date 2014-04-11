#!/usr/bin/env python


"""
# orig version

from joy import Joy
from robot import Robot

joy=Joy()
robot=Robot()
MaxSpeed = 0.6
AScale = 1.

while 1:
  joy.update()	
  robot.setSpeedPxPa( joy.Speed * MaxSpeed, -joy.Angle * AScale )
  robot.update()
"""

import sys
from joy import Joy
from eduro import EmergencyStopException, emergencyStopExtension
from eduromaxi import EduroMaxi
import launcher

import camera
from magellan import displayGpsStatusExtension
from localisation import SimpleOdometry, KalmanFilter

def parse_params(cmdlineParams):
  if cmdlineParams:
    return dict(option.split('=') for option in cmdlineParams.split(':'))
  return dict()

class Joydrive:
  def __init__( self, robot, cmdlineParams = None, verbose = False ):
    params = parse_params(cmdlineParams)

    self.robot = robot
    self.verbose = verbose
    self.robot.attachGPS()
    self.robot.attachLaser()
    cameraURL = params['url'] if 'url' in params else camera.DEFAULT_URL
    self.robot.attachCamera(cameraExe = 'cat', url=cameraURL)
    self.robot.addExtension( emergencyStopExtension )
    self.robot.addExtension( displayGpsStatusExtension )
    self.robot.localisation = SimpleOdometry()

  def __call__( self ):
    joy=Joy()
    MaxSpeed = 0.6
    AScale = 1.
    try:
      self.robot.gps.start()
      self.robot.laser.start()
      self.robot.camera.start()
      while 1:
        joy.update()	
        self.robot.setSpeedPxPa( joy.Speed * MaxSpeed, -joy.Angle * AScale )
        self.robot.update()
    except EmergencyStopException, e:
      print "EmergencyStopException"
    self.robot.gps.requestStop()
    self.robot.laser.requestStop()
    self.robot.camera.requestStop()


if __name__ == "__main__":
  launcher.launch(sys.argv, EduroMaxi, Joydrive)

