#!/usr/bin/python
"""
  Cube detector with Velodyne VLP-16
  (planned for SICK Robot Day 2016)
  usage:
       ./cube.py <task|thread> [<metalog> [<F>]]
"""
# requires source from osgar & apyros

import sys
import os
import inspect
OSGAR_ROOT = os.path.realpath(os.path.abspath(os.path.join(
    os.path.split(inspect.getfile(inspect.currentframe() ))[0], '..', 'osgar')))
if OSGAR_ROOT not in sys.path:
    assert 'eduro' in sys.path[0], sys.path
    sys.path.insert(1, OSGAR_ROOT) # access without installation
#    sys.path.append(OSGAR_ROOT)  # collision of can.py and other files
# TODO fix velodyne logging and provide apyros+sensors Python package

from velodyne import Velodyne, LASER_ANGLES
from apyros.metalog import MetaLog, disableAsserts

import numpy as np


def load_background():
#    return np.zeros((360, 16), dtype=np.uint16)  # default = no action
#    return np.full((360, 16), 1000, dtype=np.uint16)  # just for subtraction test
    return np.loadtxt('cube-background.txt', dtype=np.uint16)


def save_background(arr, filename):
    f = open(filename, 'w')
    for i in xrange(360):
        for j in xrange(16):
            f.write('{} '.format(arr[i][j]))
        f.write('\n')
    f.close()


def remove_background(scan, ground):
    mask = scan >= background
    ret = scan.copy()
    ret[mask] = 0
    return ret


def print_data(scan):
    print [x for a,x in sorted(zip(LASER_ANGLES, scan[0]))]
    print


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

