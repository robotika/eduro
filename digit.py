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

import numpy as np

DIGIT_EXE_PATH = r'm:\git\eduro\c\digits\Debug\digits.exe'
DIGIT_CWD = r'm:\git\cvdrone\bin\vs2008'
TMP_OUTPUT_PATH = r"m:\git\eduro\out.jpg"
TMP_OUTPUT_LOG = r"m:\git\eduro\out_tmp.log"

def fitsIn( (x,y,w,h), cnt ):
    "return true if rectanble fits in given contour"
    for p in cnt:
        if  x < p[0][0] < x+w and y < p[0][1] < y+h:
            return False
    return True
    
def validDigitPosition( x, y, w, h ):
    "can given bounding box contain a navigation number?"
    # ax + by + c = 0
    # points: (0,180), (240,0)
    # 180b+c = 0 and 240a + c = 0, size = 300
    v = 0.6*y + 0.8*h - 144.0
    print v
    return abs(v) < 10

def recognizeDigits( frame, level = 130 ):
    gray = cv2.cvtColor( frame, cv2.COLOR_BGR2GRAY )
    ret, binary = cv2.threshold( gray, level, 255, cv2.THRESH_BINARY )
    tmp = cv2.cvtColor( binary, cv2.COLOR_GRAY2BGR )
    contours, hierarchy = cv2.findContours( binary, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE )
    ret = False
    c = None
    for i,h in enumerate(hierarchy[0]):
        n,p,child,parent = h
        x,y,w,h = cv2.boundingRect( contours[i] )
        if 20 < w < 140 and 20 < h < 180 and w < h < 2*w:
#            print i, (x, y), (x+w, y+h), w, h
            c = i
            b = -1
            if x > b and y > b and x+w < 640-b and y+h < 512-b:
                if parent >= 0 and fitsIn( (x-b,y-b,w+2*b,h+2*b), contours[parent] ):
                    if validDigitPosition( x, y, w, h ):
                        cv2.drawContours(tmp, [contours[c]], -1, (0,255,0), 2)
                        cv2.rectangle( tmp, (x,y), (x+w,y+h), color=(0,128,255), thickness=2 )
                        ret = True
    cv2.imshow( 'bin', tmp )
    return ret




def findParent( hierarchy ):
    ret = []
    if hierarchy is None or len(hierarchy) == 0:
        return ret

    parents = defaultdict( list )
    for i,h in enumerate(hierarchy[0]):
        n,p,child,parent = h
        parents[parent].append(i)
    for (k,v) in parents.items():
        if len(v) == 12:
            print k, v, len(v)
            ret.append( (k,v) )
    return ret


def recognizeNavTargetHC( frame, level = 130 ):
    gray = cv2.cvtColor( frame, cv2.COLOR_BGR2GRAY )
    circles = cv2.HoughCircles( gray, cv2.cv.CV_HOUGH_GRADIENT, dp=1, minDist = 1 )
    for cir in circles[0]:
        cv2.circle(frame, (int(cir[0]),int(cir[1])), int(cir[2]), (0,0,255), 2 ),
    cv2.imwrite( "tmp.jpg", frame )
    cv2.imshow( 'img', frame )


def recognizeNavTarget( frame, level = 130 ):
    gray = cv2.cvtColor( frame, cv2.COLOR_BGR2GRAY )
    kernel = np.ones( (3,3), np.uint8)
    gray = cv2.erode( gray, kernel )
    ret, binary = cv2.threshold( gray, level, 255, cv2.THRESH_BINARY )
    tmp = cv2.cvtColor( binary, cv2.COLOR_GRAY2BGR )
    contours, hierarchy = cv2.findContours( binary, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE )
    ret = False
    for sel,kids in findParent( hierarchy ):
        M = cv2.moments( contours[sel] )
        if min( [cv2.contourArea( contours[c] ) for c in kids]) > 0 and M['m00'] < 15000:
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])        
            print "Center", (cx,cy), "area", M['m00']
            print sorted([cv2.contourArea( contours[c] ) for c in kids])
            areas = []
            verify = []
            for c in kids:
                x,y,w,h = cv2.boundingRect( contours[c] )
                areas.append( (w*h, c) )
                verify.append( (x,y,w,h,c) )
            areas = sorted( areas )
            bigOnes = [c for a,c in areas[8:]]
            midOnes = [c for a,c in areas[4:8]]
            print areas
            for a,c in areas[:4]:
                cv2.drawContours(tmp, [contours[c]], -1, (255,0,0), -1)   
            for a,c in areas[4:8]:
                cv2.drawContours(tmp, [contours[c]], -1, (0,220,0), -1)   
            for a,c in areas[8:]:
                cv2.drawContours(tmp, [contours[c]], -1, (0,0,220), -1)   
            # verify that extremes are the bigger sub-contours
            val, c1 = min( [ (x+y,c) for (x,y,w,h,c) in verify] )
            val, c2 = max( [ (x+w+y+h,c) for (x,y,w,h,c) in verify] )
            val, c3 = min( [ (x-y-h,c) for (x,y,w,h,c) in verify] )
            val, c4 = max( [ (x+w-y,c) for (x,y,w,h,c) in verify] )
            if sorted([c1,c2,c3,c4])==sorted(bigOnes):
                verify = [ (x,y,w,h,c) for (x,y,w,h,c) in verify if c not in bigOnes]
                val, c1 = min( [ (x+y,c) for (x,y,w,h,c) in verify] )
                val, c2 = max( [ (x+w+y+h,c) for (x,y,w,h,c) in verify] )
                val, c3 = min( [ (x-y-h,c) for (x,y,w,h,c) in verify] )
                val, c4 = max( [ (x+w-y,c) for (x,y,w,h,c) in verify] )
                if sorted([c1,c2,c3,c4])==sorted(midOnes):
                    ret = True
                else:
                    print "!!!PRESS ANY KEY!!!"
                    cv2.imshow( 'bin', tmp )
                    cv2.waitKey(100) # blocking was not good for batch processing
    cv2.imwrite( "tmp.png", tmp )
    cv2.imshow( 'bin', tmp )
    return ret


def recognizeNavTargetEx( frame, threshold, note=None ):
    "handle multiple thresholds (for negative number)"
    if threshold < 0:
        detectedAt = []
        for threshold in xrange(255):
            if recognizeNavTarget( frame, threshold ):
                detectedAt.append(threshold)
                cv2.waitKey(10)
            else:
                cv2.waitKey(1)
        print detectedAt

        if detectedAt:
            f = open("detect.txt",'a')
            if detectedAt == range(detectedAt[0], detectedAt[-1]+1):
                f.write( '%d\t%d\t' % (detectedAt[0], detectedAt[-1]) + str(note) + '\n')
            else:
                f.write( str(detectedAt) + '\t' + str(note) + '\n')
            f.close()
        return len(detectedAt) > 0
    else:
        return recognizeNavTarget( frame, threshold )


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

        if imgFile is not None:
            print imgFile
            imgAbsPath = os.path.dirname( filename ) + os.sep + imgFile.split('/')[-1]            
            tmpLog = open( TMP_OUTPUT_LOG, "w" )
            subprocess.check_call( [DIGIT_EXE_PATH, imgAbsPath, TMP_OUTPUT_PATH], cwd = DIGIT_CWD, stdout=tmpLog )
            tmpLog.close()
            tmpLog = open( TMP_OUTPUT_LOG )
            buf = tmpLog.read()
            fout.write( str( (buf, imgFile) ) + '\n' )
            tmpLog.close()
            img = cv2.imread( TMP_OUTPUT_PATH )
            cv2.imshow( 'image', img )
            if cv2.waitKey(500) >= 0:
                break
        else:
            fout.write( line )
        fout.flush()
    fout.close()


if __name__ == "__main__": 
    if len(sys.argv) < 2:
        print __doc__
        sys.exit(-1)
    path = sys.argv[1]
    threshold = 80
    if len(sys.argv) > 2:
        threshold = int(sys.argv[2])
    if path.endswith(".jpg"):
#        recognizeDigits( cv2.imread( sys.argv[1] ), threshold )
        recognizeNavTargetEx( cv2.imread( sys.argv[1] ), threshold )
        cv2.waitKey(0)
        sys.exit(0)
    if path.endswith(".log"):
        processLog( path )
        sys.exit(0)
    for (dirpath, dirnames, filenames) in os.walk(path):
        for name in filenames:
            if name.endswith(".jpg"):
                print name
#                if recognizeDigits( cv2.imread( dirpath+ os.sep+name ), threshold ):
                if recognizeNavTargetEx( cv2.imread( dirpath+ os.sep+name ), threshold, note=dirpath+os.sep+name ):
                    if cv2.waitKey(1000) != -1:
                        break
                else:
                    if cv2.waitKey(10) != -1:
                        break


#-------------------------------------------------------------------
# vim: expandtab sw=4 ts=4 

