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
from robot import SIDE_LEFT, SIDE_RIGHT
from driver import Driver, normalizeAnglePIPI, angleTo
from localisation import SimpleOdometry

from can import CAN, ReplyLog, ReplyLogInputsOnly 

import viewlog
from viewlog import viewLogExtension, viewCompassExtension, viewPoseExtension

from ray_trace import combinedPose
from line import distance, Line
from route import Route, DummyConvertor

from sdoplg import ReadSDO, WriteSDO


import starter

from cube import CubeDetector, verify_loaded_cube
import numpy as np

from cube_tools import cubesFromScan

RFU620_POSE = (-0.35, 0.14, 0)


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

#----------------------------------------------------

def draw_rfu620_extension(robot, id, data):
    if id=='rfu620':
        posXY = combinedPose(robot.localisation.pose(), RFU620_POSE)[:2]
        for index, d in enumerate(data[1], start=1):
            robot.last_valid_rfid = d, posXY
            i, rssi = d[:2]  # i.e. 0x1000 0206 0000
            x, y, zone = (i >> 24)&0xFF, (i >> 16)&0xFF, i&0xFF
            print hex(i), (x, y), rssi, '({}/{})'.format(index, len(data[1]))
            if (x + y) % 2 == 0:
                if rssi > -60:
                    viewlog.dumpBeacon(posXY, color=(0, 0, 255))
                else:
                    viewlog.dumpBeacon(posXY, color=(0, 0, 128))
            else:
                if rssi > -60:
                    viewlog.dumpBeacon(posXY, color=(255, 0, 255))
                else:
                    viewlog.dumpBeacon(posXY, color=(128, 0, 128))

#----------------------------------------------------

def draw_cubes_extension(robot, id, data):
    if id == 'laser':
#        cd = CubeDetector(robot.laser.pose)
#        cubes = cd.detect_cubes_xy(data, limit=4)
        # Jakub's alternative
        cubes = cubesFromScan(data)
        for cube_x, cube_y in cubes:
            goal = combinedPose(robot.localisation.pose(), (cube_x, cube_y, 0))[:2]
            viewlog.dumpBeacon(goal, color=(128, 255, 128))

#----------------------------------------------------

def is_path_blocked(raw_laser_data, raw_remission_data=None):
    # TODO asymetric filtering based on laser position
    # TODO use reference array for safety area
    # TODO move to separate file??
    arr = np.array(raw_laser_data, dtype=np.uint16)
    arr[arr == 0] = 10000
    if raw_remission_data is not None:
        rem_arr = np.array(raw_remission_data[:-3], dtype=np.uint16)  # what are the 3 extra values 0, 1, 11?!
        assert len(arr) == len(rem_arr), (len(arr), len(rem_arr))
        arr[rem_arr < 50] = 10000
    
    m = min(arr[135-45:135+45])
    if m < 200:
        count = sum(arr[135-45:135+45] < 200)
        print m, arr[60:-60], "count =", count
        return count >= 10
    else:
        return False


def storage_tag(tag):
    assert tag is not None
    tag_id, rssi = tag[:2]
    zone = tag_id & 0xFF
    return zone in [1, 2]


def is_in_loading_zone(pose, last_rfid):
    x, y, a = pose
    tag, tag_pose = last_rfid
    if tag is None:
        return x < 3.5 and 1.5 < y < 5.5
    # TODO verify validity distance + take into account robot offset
    return storage_tag(tag)


class SICKRobotDay2016:
    def __init__(self, robot, code, verbose = False):
        self.random = random.Random(0).uniform 
        self.robot = robot
        self.verbose = verbose
        self.code = code
        self.robot.attachEmergencyStopButton()

        # change robot wheel calibration
        wd = 427.0 / 445.0 * 0.26/4.0 # with gear  1:4
        delta = -0.00015
        self.robot._WHEEL_DIAMETER = {SIDE_LEFT: wd+delta, SIDE_RIGHT: wd-delta}

        self.robot.attachCamera(sleep=0.5)
        self.robot.attachLaser( remission=True, pose=((0.24, -0.13, 0.08), (0,math.radians(180),0)) )
        self.robot.attachRFU620()
        
        self.driver = Driver( self.robot, maxSpeed = 0.5, maxAngularSpeed = math.radians(180) )
        self.robot.localisation = SimpleOdometry()

        self.robot.last_valid_rfid = None, None  # tag, position
        self.robot.addExtension(draw_rfu620_extension)

        self.robot.addExtension(draw_cubes_extension)

        self.robot.laser.stopOnExit = False    # for faster boot-up


    def load_cube(self):
        print "load cube (time = {:.1f}s)".format(self.robot.time - self.start_time)
        self.driver.goStraight(0.3, timeout=30)
        gripperClose(self.robot)

    def place_cube(self):
        pos = combinedPose(self.robot.localisation.pose(), (0.2, 0, 0))[:2]  # TODO check proper offset
        print "place cube at ({:.2f}, {:.2f} (time = {:.1f}))".format(pos[0], pos[1], self.robot.time - self.start_time)
        viewlog.dumpBeacon(pos, color=(255, 255, 255))
        viewlog.dumpObstacles([[pos]])
        gripperOpen(self.robot)
        self.driver.goStraight(-0.3, timeout=30)

    def game_over(self):
        print 'Game over (battery status = {}V)'.format(self.robot.battery)
        for k in xrange(10):
            self.robot.setSpeedPxPa(0.0, 0.0)
            self.robot.update()
        raise EmergencyStopException() # TODO: Introduce GameOverException as in Eurobot


    def find_cube(self, timeout):
        print "find_cube-v1"
        prev = None
        cd = CubeDetector(self.robot.laser.pose)
        startTime = self.robot.time
        while self.robot.time < startTime + timeout:
            self.robot.update()
            if prev != self.robot.laserData:
                prev = self.robot.laserData
                cubes = cd.detect_cubes_xy(self.robot.laserData, limit=4)
                if len(cubes) > 0:
                    for i, (cube_x, cube_y) in enumerate(cubes):
                        goal = combinedPose(self.robot.localisation.pose(), (cube_x, cube_y, 0))[:2]
                        viewlog.dumpBeacon(goal, color=(200, 200, 0) if i > 0 else (255, 255, 0))
                    cube_x, cube_y = cubes[0]
                    cube_y += 0.05  # hack for bended(?) laser
                    print "{:.2f}\t{:.2f}".format(cube_x, cube_y)
                    speed = 0.0

                    tolerance = 0.01
                    if cube_x > 0.5:
                        tolerance = 0.05

                    angle = math.atan2(cube_y, cube_x)
                    turnStep = math.radians(10)
                    if abs(angle) > math.radians(10):
                        turnStep = math.radians(50)

                    if cube_y > tolerance:
                        angularSpeed = turnStep
                    elif cube_y < -tolerance:
                        angularSpeed = -turnStep
                    else:
                        angularSpeed = 0
                        speed = 0.1
                        if cube_x > 0.5:
                            speed = 0.3  # move faster toward further goals
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


    def handle_single_cube(self, pts, rev_pts):
        if self.find_cube(timeout=20.0):
            self.load_cube()
            print verify_loaded_cube(self.robot.laserData)
        for cmd in self.driver.followPolyLineG(pts, withStops=True):
            self.robot.setSpeedPxPa(*cmd)
            self.robot.update()
            if (not is_in_loading_zone(self.robot.localisation.pose(),
                                       self.robot.last_valid_rfid)
                and is_path_blocked(self.robot.laserData, self.robot.remissionData)):
                print "ESCAPING FROM", self.robot.localisation.pose()
                self.driver.stop()
                break
        self.place_cube()

        # handle offset in case of blocked path
        print rev_pts
        route = Route(pts=rev_pts, conv=DummyConvertor())
        _, dist, signed_index = route.findNearestEx(self.robot.localisation.pose())
        rev_pts = rev_pts[max(0, abs(signed_index) - 1):]
        print "cut path", dist, signed_index, rev_pts

        self.driver.turn(angle=math.radians(180), timeout=30)

        for cmd in self.driver.followPolyLineG(rev_pts):
            self.robot.setSpeedPxPa(*cmd)
            self.robot.update()


    def ver1(self, verbose=False):
        # Navigate on polyline
        print "ver1", self.robot.battery, self.driver.maxAngularSpeed
        self.driver.maxAngularSpeed = math.pi/2.0

        paths = [
            ( [(1.5, 2.5), (1.5, 1.5), (6.5, 1.5)], [(6.5, 1.5), (3.5, 1.5)] ),
            ( [(3.5, 2.5), (5.5, 2.5)], [(5.5, 2.5), (3.5, 2.5)] ),
            ( [(3.5, 3.5), (6.5, 3.5)], [(6.5, 3.5), (3.5, 3.5)] ),
            ( [(3.5, 4.5), (5.5, 4.5)], [(5.5, 4.5), (3.5, 4.5)] ),
            ( [(3.5, 5.5), (6.5, 5.5)], [(6.5, 5.5), (3.5, 5.5)] ),

            ( [(3.5, 5.5), (5.5, 5.5)], [(5.5, 5.5), (3.5, 5.5)] ),
            ( [(3.5, 4.5), (4.5, 4.5)], [(4.5, 4.5), (3.5, 4.5)] ),
            ( [(3.5, 3.5), (5.5, 3.5)], [(5.5, 3.5), (3.5, 3.5)] ),
            ( [(3.5, 2.5), (4.5, 2.5)], [(4.5, 2.5), (3.5, 2.5)] ),
            ( [(3.5, 1.5), (5.5, 1.5)], [(5.5, 1.5), (3.5, 1.5)] ),

            ( [(3.5, 1.5), (4.5, 1.5)], [(4.5, 1.5), (3.5, 1.5)] ),
            ( [(3.5, 3.5), (4.5, 3.5)], [(4.5, 3.5), (3.5, 3.5)] ),
            ( [(3.5, 5.5), (4.5, 5.5)], [(4.5, 5.5), (3.5, 5.5)] ),
        ]
        self.robot.localisation.setPose( (1.5, 2.2, 0.0) )
        for path, rev_path in paths:
            self.handle_single_cube(path, rev_path)
        self.game_over()


    def test_line( self, verbose=False ):
        print "test_line", self.robot.battery
        self.driver.goStraight(10.0, timeout=60)
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
        self.start_time = float('nan')  # just to be defined in case of EmergencyStopException
        try:
            if getattr( self.robot.laser, 'startLaser', None ):
                # trigger rotation of the laser, low level function, ignore for log files
                print "Powering laser ON"
                self.robot.laser.startLaser() 
            gripperOpen(self.robot)
            self.robot.waitForStart()
            self.start_time = self.robot.time

            self.robot.laser.start()    # laser also after start -- it should be already running
            self.robot.camera.start()
            self.robot.rfu620.start()
            self.robot.localisation = SimpleOdometry()

            while True:
#                self.ver0(verbose = self.verbose)
                self.ver1(verbose = self.verbose)
#                self.test_line(verbose = self.verbose)

        except EmergencyStopException, e:
            print "EmergencyStopException at {} sec".format(self.robot.time - self.start_time)
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

