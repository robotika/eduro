#!/usr/bin/env python
"""
  Field Robot Event 2015 - create map of the maize field
  usage:
       ./frelog2map.py <metalog> <output image file>
"""

#from fre import *
import sys
import cv2
import numpy as np


UNKNOWN = (0xbe, 0xbe, 0xbe) # Unknown/unexplored area: GRAY
PLANS = 0x0       # Plants/Obstacles/Walls: BLACK
FREE = 0xffffff   # Free known area/explored area: WHITE
MARKED = 0xff0000 # Marked plants/detection target: RED

def createMap( metalog ):
    img = np.zeros( (600,800,3), np.uint8 )
    cv2.rectangle( img, (0,0), (img.shape[1], img.shape[0]), UNKNOWN, -1 )
    return img
    

if __name__ == "__main__":
    if len(sys.argv) < 3:
        print __doc__
        sys.exit(2)
    img = createMap( sys.argv[1] )
    cv2.imwrite( sys.argv[2], img )

# vim: expandtab sw=4 ts=4 


