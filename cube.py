#!/usr/bin/python
"""
  Cube detector with small horizontally placed SICK laser rangefinder
  (planned for SICK Robot Day 2016)
  usage:
       ./cube.py <task|thread> [<metalog> [<F>]]
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
    metalog=None
    if 'meta_' in sys.argv[1]:
        metalog = MetaLog(filename=sys.argv[1])
    elif len(sys.argv) > 2:
        metalog = MetaLog(filename=sys.argv[2])
    if len(sys.argv) > 2 and sys.argv[-1] == 'F':
        disableAsserts()

    sensor = Velodyne(metalog=metalog)
    if sys.argv[1] == 'thread':
        thr = VelodyneThread(sensor)
        start_time = datetime.now()
        thr.start()
        prev = None
        while datetime.now() - start_time < timedelta(seconds=3.0):
            curr = thr.scan_safe_dist()
            if prev != curr:
                print curr
            prev = curr
        thr.requestStop()
        thr.join()
    else:
        background = load_background()
        prev = None
        for i in xrange(10000):
            sensor.update()
            curr = sensor.scan_index, sensor.dist_index
            if prev != curr:
                data = remove_background(sensor.dist, background)
                print_data(data)
#                if sensor.scan_index % 10 == 0:
#                    print curr
            prev = curr

# vim: expandtab sw=4 ts=4 

