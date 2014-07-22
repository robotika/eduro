#!/usr/bin/python
"""
  OSGAR - Open Source Garden Autonomous Robot 
     ... pre-collecting data via Eduro
"""

import sys
import math
import itertools
import datetime

from localisation import SimpleOdometry
from driver import Driver, normalizeAnglePIPI, angleTo

from eduro import EmergencyStopException, emergencyStopExtension
from hand import setupHandModule


class Osgar:
    def __init__( self, robot, configFilename, verbose = False ):
        self.robot = robot
        self.robot.fnSpeedLimitController = [] 
        self.robot.addExtension( emergencyStopExtension )
        self.robot.attachGPS()
        self.robot.attachLaser( remission=True )
        self.robot.laser.stopOnExit = False  # for faster boot-up
        self.robot.attachCamera( cameraExe = "../robotchallenge/rc" ) # TODO what was used?!
        self.robot.attachHand()
        self.robot.gpsData = None
        self.driver = Driver( self.robot, maxSpeed = 0.7, maxAngularSpeed = math.radians(60) )
        self.robot.localisation = SimpleOdometry()
        self.verbose = verbose
        self.configFilename = configFilename

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
            self.robot.toDisplay += '-'
            self.robot.update()
        print "!!! GO !!!" 

    def ver0( self ):
        print "Data collector ..."
        try:
            # start GPS sooner to get position fix
            self.robot.gps.start()
            self.robot.laser.start()
            self.waitForStart()
            self.robot.camera.start()
            print "battery:", self.robot.battery
            startTime = self.robot.time
            while True:
                self.robot.setSpeedPxPa( 0.0, 0.0 )
                self.robot.update()
        except EmergencyStopException, e:
	        print "EmergencyStopException"
        print "battery:", self.robot.battery
        self.robot.camera.requestStop()
        self.robot.gps.requestStop()
        self.robot.laser.requestStop()

    def __call__( self ):
        print "OSGAR RUNNING:", self.configFilename
        if self.configFilename.startswith("cmd:"):
            return eval( self.configFilename[4:] )
        return self.ver0()

from eduromaxi import EduroMaxi
import launcher
if __name__ == "__main__": 
    launcher.launch( sys.argv, EduroMaxi, Osgar, configFn=setupHandModule )

# vim: expandtab sw=4 ts=4

