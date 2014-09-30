#!/usr/bin/python
"""
  Digit recognition - SICK Robot Day 2014
    usage:
         ./digit.py <img filename|directory|log file>
"""

import sys
import os
import cv2
from collections import defaultdict
import subprocess

DIGIT_EXE_PATH = r'm:\git\eduro\c\digits\Debug\digits.exe'
DIGIT_CWD = r'm:\git\cvdrone\bin\vs2008'
TMP_OUTPUT_PATH = r"m:\git\eduro\out.jpg"

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


def processLog( filename ):
    f = open(filename)
#    console = subprocess.Popen( [DIGIT_EXE_PATH,] , cwd = DIGIT_CWD, stdin=subprocess.PIPE, stdout=subprocess.PIPE )

    fout = open("out.txt","w")
    while True:
        num = f.readline()
        fout.write(num)
        fout.flush()
        line = f.readline()
        if len(line) == 0:
            break
        cmd,imgFile = eval(line)
        if imgFile:
            fout.write( imgFile.strip() + ' ' )
        else:
            fout.write( str(imgFile)+'\n' )
        fout.flush()

        if imgFile is not None:
            print imgFile
            imgAbsPath = os.path.dirname( filename ) + os.sep + imgFile.split('/')[-1]            
            subprocess.check_call( [DIGIT_EXE_PATH, imgAbsPath, TMP_OUTPUT_PATH], cwd = DIGIT_CWD, stdout=fout )
            img = cv2.imread( TMP_OUTPUT_PATH )
            cv2.imshow( 'image', img )
            if cv2.waitKey(200) >= 0:
                break
    fout.close()


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
    if path.endswith(".log"):
        processLog( path )
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

