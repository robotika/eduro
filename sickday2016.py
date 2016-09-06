#!/usr/bin/python
"""
  SICK Robot Day 2016 main program
    usage:
       ./sickday.py <note or cmd:<test command>> [--file <log file> [F|FF]]
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
from eduromaxi import EduroMaxi
from driver import Driver, normalizeAnglePIPI, angleTo
from localisation import SimpleOdometry

from can import CAN, ReplyLog, ReplyLogInputsOnly 

import viewlog
from viewlog import viewLogExtension, viewCompassExtension, viewPoseExtension

from ray_trace import combinedPose
from line import distance, Line
from route import loadLatLonPts, Convertor

from sdoplg import ReadSDO, WriteSDO


import starter


def setupGripperModule(can): 
    writer = WriteSDO( 0x7F, 0x2100, 1, [0xF] )  # enable servos
    for cmd in writer.generator():
        if cmd:
            can.sendData( *cmd )
        writer.update( can.readPacket() ) 


def gripperServo(can, left, right):
    can.sendData( 0x37F, [0,0,0,0, left & 0xFF, (left >> 8)&0xFF, right & 0xFF, (right >> 8)&0xFF] )  


class SICKRobotDay2016:
    def __init__(self, robot, code, verbose = False):
        self.random = random.Random(0).uniform 
        self.robot = robot
        self.verbose = verbose
        self.code = code
        self.robot.attachEmergencyStopButton()

        # do we want some processing?
#        self.robot.attachCamera( cameraExe = "../digits/digits", 
#                url = "http://192.168.0.99/image?res=full&x0=352&y0=80&x1=992&y1=592&quality=12&doublescan=0" )
        self.robot.attachCamera(sleep=0.5)
#        self.robot.addExtension( cameraDataExtension )
#        self.robot.attachLaser( pose=((0.14, 0.0, 0.32), (0,0,0)) )
#        self.robot.attachLaser( index=2, remission=True, usb=True, 
#                pose=((0.19, 0.0, 0.055), tuple([math.radians(x) for x in (0, 180, 0)])), 
#                errLog = self.robot.metaLog )
        
        self.driver = Driver( self.robot, maxSpeed = 0.5, maxAngularSpeed = math.radians(180) )
        self.robot.localisation = SimpleOdometry()
        
#        self.robot.laser.stopOnExit = False    # for faster boot-up


    def run( self ):
        try:
#            if getattr( self.robot.laser, 'startLaser', None ):
                # trigger rotation of the laser, low level function, ignore for log files
#                print "Powering laser ON"
#                self.robot.laser.startLaser() 
            self.robot.waitForStart()
#            self.robot.laser.start()    # laser also after start -- it should be already running
#            self.robot.laser2.start() 
            self.robot.camera.start()
            self.robot.localisation = SimpleOdometry()
            while True:
                self.ver0(verbose = self.verbose)            
        except EmergencyStopException, e:
            print "EmergencyStopException"
#        self.robot.laser.requestStop()
#        self.robot.laser2.requestStop() 
        self.robot.camera.requestStop()


    def ver0( self, verbose=False ):
        # Go straight for 2 meters
        print "ver0", self.robot.battery

        gripperServo(self.robot.can, 32512, 51200)  # open
        self.driver.goStraight(0.3, timeout=30)
        gripperServo(self.robot.can, 43520, 32512)  # close
        self.driver.goStraight(2.0, timeout=30)
        gripperServo(self.robot.can, 32512, 51200)  # open
        self.driver.goStraight(-0.3, timeout=30)

        print 'Game over.', self.robot.battery
        for k in xrange(10):
            self.robot.setSpeedPxPa(0.0, 0.0)
            self.robot.update()
        raise EmergencyStopException() # TODO: Introduce GameOverException as in Eurobot


    def __call__( self ):
        print "RUNNING:", self.code
        if self.code.startswith("cmd:"):
            return eval(self.code[4:]) 
        return self.run()


if __name__ == "__main__": 
    from eduromaxi import EduroMaxi
    import launcher
    launcher.launch(sys.argv, EduroMaxi, SICKRobotDay2016, configFn=setupGripperModule)

# vim: expandtab sw=4 ts=4 

