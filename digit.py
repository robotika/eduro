#!/usr/bin/python
"""
  Digit recognition - SICK Robot Day 2014
    usage:
         ./digit.py <img filename>
"""

import sys
import os
import cv2

g_mser = None

def recognizeDigits( frame ):
    global g_mser
    gray = cv2.cvtColor( frame, cv2.COLOR_BGR2GRAY )
    if g_mser == None:
        g_mser = cv2.MSER( _delta = 1, _min_area=100, _max_area=3000 )
    contours = g_mser.detect(gray, None)
    cv2.drawContours(frame, contours, -1, (0,255,0), 3)
    cv2.imshow( 'image', frame )
    level = 30
    ret, binary = cv2.threshold( gray, level, 255, cv2.THRESH_BINARY )
    cv2.imshow( 'bin', binary )

if __name__ == "__main__": 
    if len(sys.argv) < 2:
        print __doc__
        sys.exit(-1)
    path = sys.argv[1]
    if path.endswith(".jpg"):
        recognizeDigits( cv2.imread( sys.argv[1] ) )
        cv2.waitKey(0)
        sys.exit(0)
    for name in os.listdir(path):
        if name.endswith(".jpg"):
            recognizeDigits( cv2.imread( path+ os.sep+name ) )
            if cv2.waitKey(100) != -1:
                break


#-------------------------------------------------------------------
# vim: expandtab sw=4 ts=4 

