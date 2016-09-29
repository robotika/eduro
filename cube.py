#!/usr/bin/python
"""
  Cube detector with small horizontally placed SICK laser rangefinder
  (planned for SICK Robot Day 2016)
  usage:
       ./cube.py <src_laser file>
"""
import sys
import numpy as np
import math

def detect_cubes_v0(raw_laser_data, verbose=False):
    arr = np.array(raw_laser_data)
    mask = arr < 10  # 1cm "blindnesss"
    arr[mask] = 10000
    blind_offset = 45
    index = np.argmin(arr[blind_offset:]) + blind_offset
    if arr[index] < 2000:  # 2 meters
        left_index = right_index = index
        cube_max_dist = arr[index] + 110
        while left_index > 0 and arr[left_index] < cube_max_dist:
            left_index -= 1
        while right_index < len(arr) and arr[right_index] < cube_max_dist:
            right_index += 1
        center_index = (left_index + right_index)/2
        cube_size = math.radians(right_index - left_index) * arr[center_index]/1000.0
        if verbose:
            print left_index, index, right_index, '->', center_index
            print arr[left_index:right_index+1]
            print "CUBE SIZE", cube_size
        if 0.1 < cube_size < 0.25:
            return [(center_index, arr[center_index])]
    return []


class CubeDetector:

    def __init__(self, laser_pose_6D):
        self.laser_x = laser_pose_6D[0][0]
        self.laser_y = laser_pose_6D[0][1]
    
    def detect_cubes_xy(self, raw_laser_data, verbose=False):
        """return list of cubes coordinates relative to robot position"""
        ret = []
        for deg_angle, mm_dist in detect_cubes_v0(raw_laser_data, verbose=verbose):
             angle, dist = math.radians(135-deg_angle), mm_dist/1000.0
             cube_x, cube_y = self.laser_x + math.cos(angle)*dist, self.laser_y + math.sin(angle)*dist
             ret.append((cube_x, cube_y))
             if verbose:
                 print "{:.2f}\t{:.2f}".format(cube_x, cube_y)
        return ret


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print __doc__
        sys.exit(2)

    cd = CubeDetector(((0.24, -0.13, 0.08), (0,math.radians(180),0)))
    for i, line in enumerate(open(sys.argv[1]), start=1):
        if '[' in line:
            laser_data = eval(line)
            cubes = cd.detect_cubes_xy(laser_data)
            if len(cubes) == 0:
                print i
                cd.detect_cubes_xy(laser_data, verbose=True)



# vim: expandtab sw=4 ts=4 

