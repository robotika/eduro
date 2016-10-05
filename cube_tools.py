#!/usr/bin/python
"""
  Sick robot day 2016
  usage:
       python cube_tools.py [im <image file>|l <laser log> <index>]
           
"""
import sys
try:
    import cv2
except ImportError:
    pass
    
import numpy as np


color = "r" # "y"


def getCubeCenter( dist, ang, ver = 0, test = False ):
    center = None
    xArr, yArr = getCoordinates(dist, ang)
    if ver == 0: # the center of a conecting line between the firs and the last point.
        center = xArr[0] + ( xArr[-1] - xArr[0] )/2.0, yArr[0] + ( yArr[-1] - yArr[0] )/2.0
        print center
    
    elif ver == 1: #unreliable yet
        #edge detection
        step = 5
        slopeList = []
        for ii in range( 0, len(xArr), 2):
            xItem = xArr[ii : ii+step]
            yItem = yArr[ii : ii+step]
            a, b = np.polyfit(xItem, yItem, 1)
            slopeList.append(a)
        slopeArr = np.array(slopeList)
        angArr = np.arctan(slopeArr)
        diffArr = abs(np.diff( angArr ))
        edgeId = np.argmax( diffArr )
        edgeId = edgeId*2+3
        print angArr
        #print edgeId
        #choose a better side
        if edgeId < len(xArr)/2:
            cubeSideX = xArr[edgeId:]
            cubeSideY = yArr[edgeId:]
        else:
            cubeSideX = xArr[:edgeId]
            cubeSideY = yArr[:edgeId]
        #for cube orientation?
        #sideCen = cubeSideX[0] + ( cubeSideX[-1] - cubeSideX[0] )/2.0, cubeSideY[0] + ( cubeSideY[-1] - cubeSideY[0] )/2.0
        #print sideCen
        
        dx = ( cubeSideX[-1] - cubeSideX[0] ) *np.sqrt(2.0)/2
        dy = ( cubeSideY[-1] - cubeSideY[0] ) *np.sqrt(2.0)/2
        ra = np.pi/4
        cubeCen = cubeSideX[0] + dx*np.cos(ra) - dy*np.sin(ra), cubeSideY[0] + dx*np.sin(ra) + dy*np.cos(ra)
        #print cubeCen
        
    
    if test:
        import matplotlib.pyplot as plt
        plt.figure()
        plt.axis('equal')
        plt.plot( xArr, yArr, "bo" )
        if ver == 1:
            plt.plot( cubeSideX, cubeSideY, "ro")
            #plt.plot( sideCen[0], sideCen[1], "y+", ms = 15)
            plt.plot( cubeCen[0], cubeCen[1], "r+", ms = 15)
        if center:
            plt.plot( center[0], center[1], "r+", ms = 15 )
        plt.show()
    if ver == 1:
        return cubeCen
    return center


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


def getCoordinates(dist, ang, laserXY = None):
    #print type(dist)
    if type(dist) == list:
        dist = np.array(dist)
    if type(ang) == list:
        ang = np.array(ang)
    X = np.cos( np.radians( ang ) ) * dist
    Y = np.sin( np.radians( -ang ) ) * dist
    if laserXY:
        X = X + laserXY[0]
        Y = Y + laserXY[1]
    return X, Y
    

def cubesFromScan( scan, maxDist = 2.0, minDiff = 0.1, cubeSize = 0.16, laserXY = [0, 0] , test = False ):
    myCubeCenter = []
    distAr = np.array(scan[40:])/1000.0 # 0:40 -> only robot, no cube TODO
    angAr = np.arange( -135, 136.0 )
    angAr = angAr[40:]
    distAr[ distAr > maxDist] = np.nan
    diffAr = np.diff( distAr )
    #print distAr
    #print diffAr
    barriers = []
    itemD = []
    itemA = []
    for ii, dd in enumerate(diffAr):
        itemD.append( distAr[ii] )
        itemA.append( angAr[ii] )
        if abs(dd) > minDiff or np.isnan(dd):
            if len(itemD) > 1:
                barriers.append( [itemD, itemA] )
            itemD = []
            itemA = []
    itemD.append( distAr[ii] )
    itemA.append( angAr[ii] )
    if len(itemD) > 1:
        barriers.append( [itemD, itemA] )
        
    cubes = []
    for bar, ang in barriers:
        x0, y0 = getCoordinates( bar[0], ang[0] )
        x1, y1 = getCoordinates( bar[-1], ang[-1] )
        pointDist = np.linalg.norm( [ x0 - x1, y0 - y1 ] )
        #print pointDist
        
        if pointDist > 0.8 * cubeSize and pointDist < 1.5 * cubeSize:
            cubes.append( [bar, ang] )
            
    if cubes:
        cubeDist = []
        for cub in cubes:
            cubeDist.append( min(cub[0]) )

        idCub = np.argmin( cubeDist )
        myCube = cubes[idCub]
        myCubeCenter.append( getCubeCenter( myCube[0], myCube[1], ver = 0, test = test ) )
        
        
    if test:
        import matplotlib.pyplot as plt
        #print distAr, angAr, diffAr
        #print barriers
        corX, corY = getCoordinates( distAr, angAr, laserXY )
        #print corX, corY
        plt.figure(figsize = (10,10))
        plt.axis('equal')
        plt.plot(0,0, "k+", ms = 20)
        plt.plot(corX, corY, "o-") #RuntimeWarning: ... ?
        for d, a in barriers:
            #print d, a
            #print "#########"
            d = np.array(d)
            a = np.array(a)
            x, y = getCoordinates( d, a, laserXY )
            #print x, y
            plt.plot(x, y, "ro-")
        for cD, cA in cubes:
            cD = np.array(cD)
            cA = np.array(cA)
            x, y = getCoordinates( cD, cA, laserXY )
            plt.plot(x, y, "go-")
        x, y = getCoordinates( myCube[0], myCube[1], laserXY )
        plt.plot(x, y, "ko-")
        plt.plot( myCubeCenter[0] + laserXY[0], myCubeCenter[1] + laserXY[1], "k+", ms = 20 )
        plt.show()
    
    return myCubeCenter


def checkLog( logFile, num ):
    f = open(logFile, "r")
    ii = 0
    for line in f:
        if line[0] == "[":
            if ii != num:
                ii += 1
                continue
            scan = eval(line)
            target = cubesFromScan( scan, test = True )
            print target
            break


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
        scanNum = int(sys.argv[3])
        checkLog( logFile, scanNum )

