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

from cube import detect_cubes
import numpy as np

def setupGripperModule(can):
    writer = WriteSDO( 0x7F, 0x2100, 1, [0xF] )  # enable servos
    for cmd in writer.generator():
        if cmd:
            can.sendData( *cmd )
        writer.update( can.readPacket() ) 

def gripperDisableServos(can):
    print "gripperDisableServos() called"
    writer = WriteSDO( 0x7F, 0x2100, 1, [0x0] )  # disable servos
    for cmd in writer.generator():
        if cmd:
            can.sendData( *cmd )
        writer.update( can.readPacket() ) 
    reader = ReadSDO( 0x7F, 0x2100, 1 )
    for packet in reader.generator():
        if packet != None:
            can.sendData( *packet )
        reader.update( can.readPacket() )
    print "RESULT DATA (after):", reader.result 


def gripperServo(can, left, right):
    can.sendData( 0x37F, [0,0,0,0, left & 0xFF, (left >> 8)&0xFF, right & 0xFF, (right >> 8)&0xFF] )  

def gripperOpen(robot):
    gripperServo(robot.can, 32512, 51200)

def gripperClose(robot):
    gripperServo(robot.can, 43520, 32512)


def is_path_blocked(raw_laser_data, raw_remission_data):
    # TODO asymetric filtering based on laser position
    # TODO use reference array for safety area
    # TODO move to separate file??
    arr = np.array(raw_laser_data, dtype=np.uint16)
    rem_arr = np.array(raw_remission_data[:-3], dtype=np.uint16)  # what are the 3 extra values 0, 1, 11?!
    assert len(arr) == len(rem_arr), (len(arr), len(rem_arr))
    arr[arr == 0] = 10000
    arr[rem_arr < 50] = 10000
    
    m = min(arr[135-45:135+45])
    if m < 200:
        print m, arr[60:-60]
    return m < 200


def is_in_loading_zone(pose):
    x, y, a = pose
    return x < 1.0 and -1.5 < y < 1.5 # TODO setup proper boxes


class SICKRobotDay2016:
    def __init__(self, robot, code, verbose = False):
        self.random = random.Random(0).uniform 
        self.robot = robot
        self.verbose = verbose
        self.code = code
        self.robot.attachEmergencyStopButton()

        self.robot.attachCamera(sleep=0.5)
        self.robot.attachLaser( remission=True, pose=((0.24, -0.13, 0.08), (0,math.radians(180),0)) )
        self.robot.attachRFU620()
        
        self.driver = Driver( self.robot, maxSpeed = 0.5, maxAngularSpeed = math.radians(180) )
        self.robot.localisation = SimpleOdometry()
        
        self.robot.laser.stopOnExit = False    # for faster boot-up


    def load_cube(self):
        print "load cube"
        self.driver.goStraight(0.3, timeout=30)
        gripperClose(self.robot)

    def place_cube(self):
        print "place cube"
        gripperOpen(self.robot)
        self.driver.goStraight(-0.3, timeout=30)

    def game_over(self):
        print 'Game over (battery status = {}V)'.format(self.robot.battery)
        for k in xrange(10):
            self.robot.setSpeedPxPa(0.0, 0.0)
            self.robot.update()
        raise EmergencyStopException() # TODO: Introduce GameOverException as in Eurobot


    def find_cube0(self, timeout):
        print "find_cube"
        prev = None
        cubes = []
        goal = None
        gen = self.driver.goStraightG(1.0)
        startTime = self.robot.time
        while self.robot.time < startTime + timeout:
            for cmd in gen:
                self.robot.setSpeedPxPa(*cmd)
                self.robot.update()
                if prev != self.robot.laserData:
                    prev = self.robot.laserData
                    cubes = detect_cubes(self.robot.laserData)
                    if len(cubes) > 0:
                        deg_angle, mm_dist = cubes[0]
                        angle, dist = math.radians(135-deg_angle), mm_dist/1000.0
                        pos = self.robot.laser.pose[0][0] + math.cos(angle)*dist, self.robot.laser.pose[0][1] + math.sin(angle)*dist, 0
                        cube_x, cube_y = pos[:2]
                        print "{:.2f}\t{:.2f}".format(cube_x, cube_y)
                        goal = combinedPose(self.robot.localisation.pose(), pos)[:2]
                        viewlog.dumpBeacon(goal, color=(255, 255, 0)) 
                        if 0.0 < cube_x < 0.4 and -0.1 < cube_y < 0.1:
                            print cube_x, cube_y
                            self.robot.setSpeedPxPa(0, 0)
                            self.robot.update()
                            return True
                        gen = self.driver.goToG(goal, finishRadius=0.35, angularSpeed=math.radians(45), angleThreshold=math.radians(45))
                        break
            else:
                # no generator, and old was reached
                print "Generator Terminated!"
                return True  #???
            self.robot.update()
        print "TIMEOUT"
        return False


    def find_cube(self, timeout):
        print "find_cube-v1"
        prev = None
        startTime = self.robot.time
        while self.robot.time < startTime + timeout:
            self.robot.update()
            if prev != self.robot.laserData:
                prev = self.robot.laserData
                cubes = detect_cubes(self.robot.laserData)
                if len(cubes) > 0:
                    deg_angle, mm_dist = cubes[0]
                    angle, dist = math.radians(135-deg_angle), mm_dist/1000.0
                    pos = self.robot.laser.pose[0][0] + math.cos(angle)*dist, self.robot.laser.pose[0][1] + math.sin(angle)*dist, 0
                    cube_x, cube_y = pos[:2]
                    print "{:.2f}\t{:.2f}".format(cube_x, cube_y)
                    goal = combinedPose(self.robot.localisation.pose(), pos)[:2]
                    viewlog.dumpBeacon(goal, color=(255, 255, 0))
                    speed = 0.0
                    if cube_y > 0.01:
                        angularSpeed = math.radians(10)
                    elif cube_y < -0.01:
                        angularSpeed = math.radians(-10)
                    else:
                        angularSpeed = 0
                        speed = 0.1
                        if cube_x < 0.30:
                            return True
                    self.robot.setSpeedPxPa(speed, angularSpeed)
                else:
                    self.robot.setSpeedPxPa(0.0, 0.0)

        print "TIMEOUT"
        return False



    def ver0( self, verbose=False ):
        # Go straight for 2 meters
        print "ver0", self.robot.battery

        prevRFID = None
        self.load_cube()
        for cmd in self.driver.goStraightG(2.0):
            self.robot.setSpeedPxPa(*cmd)

            if prevRFID != self.robot.rfu620Data:
#                print self.robot.rfu620Data
                prevRFID = self.robot.rfu620Data
                posXY = combinedPose(self.robot.localisation.pose(), (-0.35, 0.14, 0))[:2]
                for d in self.robot.rfu620Data[1]:
                    i = d[0]  # i.e. 0x1000 0206 0000
                    x, y, zone = (i >> 24)&0xFF, (i >> 16)&0xFF, i&0xFF
                    print hex(i), (x, y)
                    if x == 1:
                        viewlog.dumpBeacon(posXY, color=(0, 0, 255))
                    elif x == 2:
                        viewlog.dumpBeacon(posXY, color=(255, 0, 255))
                    else:
                        viewlog.dumpBeacon(posXY, color=(255, 255, 255))
            self.robot.update()
        self.place_cube()
        self.game_over()


    def handle_single_cube(self, pts):
        if self.find_cube(timeout=20.0):
            self.load_cube()
        for cmd in self.driver.followPolyLineG(pts, withStops=True):
            self.robot.setSpeedPxPa(*cmd)
            self.robot.update()
            if (not is_in_loading_zone(self.robot.localisation.pose())
                and is_path_blocked(self.robot.laserData, self.robot.remissionData)):
                print "ESCAPING FROM", self.robot.localisation.pose()
                break
        self.place_cube()

        # TODO handle offset in case of blocked path

        self.driver.turn(angle=math.radians(180), timeout=30)

        pts.reverse()  # return path home
        for cmd in self.driver.followPolyLineG(pts):
            self.robot.setSpeedPxPa(*cmd)
            self.robot.update()


    def ver1(self, verbose=False):
        # Navigate on polyline
        print "ver1", self.robot.battery, self.driver.maxAngularSpeed
        self.driver.maxAngularSpeed = math.pi/2.0

        paths = [
            [(0, 0), (1.0, 0), (1.0, 1.0), (2.0, 1.0)],
            [(0, 0), (2.0, 0.0)],
            [(0, 0), (1.0, 0), (1.0, -1.0), (2.0, -1.0)],
        ]
        for path in paths:
            self.handle_single_cube(path)
        self.game_over()


    def test_square( self, verbose=False ):
        print "test_square", self.robot.battery
        while True:
            self.driver.turn(angle=math.radians(90), timeout=30)
            self.driver.goStraight(1.0, timeout=30)


    def test_pick_cube(self, verbose=False):
        print "test_pick_cube", self.robot.battery
        if self.find_cube(timeout=20.0):
            self.load_cube()
        self.driver.goStraight(0.5, timeout=10)
        self.place_cube()
        self.game_over()


    def run( self ):
        try:
            if getattr( self.robot.laser, 'startLaser', None ):
                # trigger rotation of the laser, low level function, ignore for log files
                print "Powering laser ON"
                self.robot.laser.startLaser() 
            gripperOpen(self.robot)
            self.robot.waitForStart()
            start_time = self.robot.time

            self.robot.laser.start()    # laser also after start -- it should be already running
            self.robot.camera.start()
            self.robot.rfu620.start()
            self.robot.localisation = SimpleOdometry()

            while True:
#                self.ver0(verbose = self.verbose)
                self.ver1(verbose = self.verbose)
#                self.test_square(verbose = self.verbose)
#                self.test_pick_cube(verbose = self.verbose)

        except EmergencyStopException, e:
            print "EmergencyStopException at {} sec".format(self.robot.time - start_time)
        gripperDisableServos(self.robot.can)
        self.robot.laser.requestStop()
        self.robot.rfu620.requestStop()
        self.robot.camera.requestStop()

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

