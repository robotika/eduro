#!/usr/bin/python
"""
  Digit recognition - SICK Robot Day 2014
    usage:
         ./digit.py <img filename>
"""

import sys
import os
import cv2
from collections import defaultdict

g_mser = None

def recognizeDigits( frame ):
    global g_mser
    gray = cv2.cvtColor( frame, cv2.COLOR_BGR2GRAY )
    if g_mser == None:
        g_mser = cv2.MSER( _delta = 1, _min_area=100, _max_area=3000 )
    contours = g_mser.detect(gray, None)
    cv2.drawContours(frame, contours, -1, (0,255,0), 3)
    cv2.imshow( 'image', frame )
    ret, binary = cv2.threshold( gray, level, 255, cv2.THRESH_BINARY )
    cv2.imshow( 'bin', binary )

def findParent( hierarchy ):
    parents = defaultdict( list )
    for i,h in enumerate(hierarchy[0]):
        n,p,child,parent = h
        parents[parent].append(i)
    ret = []
    for (k,v) in parents.items():
        if len(v) > 10 and len(v) <= 12:
            print k, v, len(v)
            ret.append( k )
    return ret


def recognizeNavTarget( frame, level = 130 ):
    gray = cv2.cvtColor( frame, cv2.COLOR_BGR2GRAY )
    ret, binary = cv2.threshold( gray, level, 255, cv2.THRESH_BINARY )
    contours, hierarchy = cv2.findContours( binary, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE )
    for sel in findParent( hierarchy ):
        cv2.drawContours(frame, [contours[sel]], -1, (0,255,0), 2)   
        M = cv2.moments( contours[sel] )
        cx = int(M['m10']/M['m00'])
        cy = int(M['m01']/M['m00'])        
        print "Center", (cx,cy)
#    for cnt in contours:
#        print cnt
#      area = cv2.contourArea(cnt, oriented=False)
#      print area

#    cv2.imshow( 'bin', binary )
    cv2.imshow( 'image', frame )

if __name__ == "__main__": 
    if len(sys.argv) < 2:
        print __doc__
        sys.exit(-1)
    path = sys.argv[1]
    if path.endswith(".jpg"):
#        recognizeDigits( cv2.imread( sys.argv[1] ) )
        recognizeNavTarget( cv2.imread( sys.argv[1] ), int(sys.argv[2]) )
        cv2.waitKey(0)
        sys.exit(0)
    for name in os.listdir(path):
        if name.endswith(".jpg"):
            print name
#            recognizeDigits( cv2.imread( path+ os.sep+name ) )
            recognizeNavTarget( cv2.imread( path+ os.sep+name ) )
            if cv2.waitKey(300) != -1:
                break


#-------------------------------------------------------------------
# vim: expandtab sw=4 ts=4 

