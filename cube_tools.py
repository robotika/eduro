#!/usr/bin/python
"""
  Sick robot day 2016
  usage:
       python tools.py <image file>
           
"""
import sys
import cv2
import numpy as np
import matplotlib.pyplot as plt

color = "r" # "y"
cubeSize = 0.16


def getOutlines( regions ):
    contours = []
    for item in regions:
        contours.append( cv2.convexHull(item.reshape(-1, 1, 2)) )
    return contours


def findCubes( img, color ):
    b,g,r = cv2.split( img )
    gray  = r
    #gray = cv2.cvtColor( img, cv2.COLOR_BGR2GRAY )
    print img.shape
    mser = cv2.MSER( _delta = 5, _min_area=16, _max_area=160 )
    contours = mser.detect(gray, None)
    contours = getOutlines( contours )
    print "number of cnt: ", len(contours)
    
    rContours = []
    for cnt in contours:
        perimeter = cv2.arcLength(cnt,True)
        ro = perimeter/2.0/np.pi
        area = cv2.contourArea(cnt)
        ra = np.sqrt( area/np.pi )
        print perimeter, ro, area, ra, ro/ra
        if ro < 1.15*ra:
            rContours.append( cnt )
            
    print "number of rCnt: ", len(rContours)
    rContours2 = []
    for cnt in rContours:
        mask = np.zeros(gray.shape,np.uint8)
        cv2.drawContours(mask,[cnt],0,255,-1)
        meanIntensity = cv2.mean(gray, mask = mask)[0]
        print meanIntensity
        if meanIntensity < 100:
            rContours2.append(cnt)
        
    print "number of rCnt2: ", len(rContours2)
    
    cv2.drawContours(img, contours, -1, (0,255,0), 1)
    cv2.drawContours(img, rContours, -1, (255,0,0), 1)
    cv2.drawContours(img, rContours2, -1, (0,0,255), 1)
    cv2.imwrite( "img3.png", img )


def checkLog( logFile ):
    f = open(logFile, "r")
    for line in f:
        if line[0] == "[":
            scan = eval(line)
            cubesFromScan( scan, test = True )
            sys.exit()
            

def getCoordinates(dist, ang):
    X = np.cos( np.radians( ang ) ) * dist
    Y = np.sin( np.radians( -ang ) ) * dist
    return X, Y
    


def cubesFromScan( scan, maxDist = 2.0, minDiff = 0.05, test = False ):
    distAr = np.array(scan[40:])/1000.0
    angAr = np.arange( -95.0, 136.0 )
    distAr[ distAr > maxDist] = np.nan
    diffAr = np.diff( distAr )
    barriers = []
    itemD = []
    itemA = []
    ii = 0
    for dd in diffAr:
        if dd > minDiff or np.isnan(dd):
            itemD.append( distAr[ii] )
            itemA.append( angAr[ii] )
            if len(itemD) > 1:
                barriers.append( [itemD, itemA] )
            itemD = []
            itemA = []
            
        else:
            itemD.append( distAr[ii] )
            itemA.append( angAr[ii] )
        ii += 1
        
    cubes = []
    for bar, ang in barriers:
        x0, y0 = getCoordinates( bar[0], ang[0] )
        x1, y1 = getCoordinates( bar[-1], ang[-1] )
        pointDist = np.linalg.norm( [ x0 - x1, y0 - y1 ] )
        print pointDist
        
        if pointDist > 0.8*cubeSize and pointDist < 1.5*cubeSize:
            cubes.append( [bar, ang] )
    
    if test:
        #print distAr, angAr, diffAr
        #print barriers
        corX = np.cos( np.radians( angAr ) ) * distAr
        corY = np.sin( np.radians( -angAr ) ) * distAr
        #print corX, corY
        plt.figure(figsize = (10,10))
        plt.plot(corX, corY, "o-")
        for d, a in barriers:
            #print d, a
            d = np.array(d)
            a = np.array(a)
            x = np.cos( np.radians( a ) ) * d
            y = np.sin( np.radians( -a ) ) * d
            #print x, y
            plt.plot(x, y, "ro-")
        for cD, cA in cubes:
            cD = np.array(cD)
            cA = np.array(cA)
            x, y = getCoordinates( cD, cA )
            plt.plot(x, y, "go-")
        plt.show()



if __name__ == "__main__":
    if len(sys.argv) < 3:
        print __doc__
        sys.exit(2)
    switch = sys.argv[1]
    if switch == "im":
        imF = sys.argv[2]
        im = cv2.imread( imF, 1 )
        findCubes( im, color )
    elif switch == "l":
        logFile = sys.argv[2]
        checkLog( logFile )
