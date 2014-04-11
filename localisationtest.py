#!/usr/bin/python

import unittest
from localisation import *

class ScoreTest( unittest.TestCase ):

  def assertEqualPose( self, p, q, places=6 ):
    self.assertAlmostEqual( p[0], q[0], places )
    self.assertAlmostEqual( p[1], q[1], places )
    self.assertAlmostEqual( p[2], q[2], places )

  def setUp( self ):
    pass

  def testKalmanFilter( self ):
    loc = KalmanFilter()
    self.assertEqualPose( loc.pose(), (0,0,0) )
    loc.setPxPa( 1.0, 0.0 )
    self.assertEqualPose( loc.pose(), (1.0,0,0) )
    loc.updateCompass( 0 ) # North
    self.assertEqualPose( loc.pose(), (1.0,0,math.pi/2) )
    loc.updateCompass( 900 ) # East
    self.assertEqualPose( loc.pose(), (1.0,0,math.pi/4) ) # NW (average)
    loc.setPxPa( 0.0, -math.pi/4 )
    self.assertEqualPose( loc.pose(), (1.0,0,0) )
    loc.updateCompass( 900 ) # East
    self.assertEqualPose( loc.pose(), (1.0,0,0) )

    loc.updateGPS( (123,456,9,1.1) )
    self.assertEqualPose( loc.pose(), (123.0,456.0,0) )
    loc.updateGPS( (125,458,9,1.1) )
    self.assertEqualPose( loc.pose(), (124.0,457.0,0) )


  def testSimplePositiveRoute( self ):
    return
    s = (16.606789, 49.204815) # I seg
    f = (16.607101, 49.205705) # Z seg
    self.assert_( score( self.route, s, f ) > 0 )
    self.assertAlmostEqual( 10.17, score( self.route, s, f), 1 )
    self.assert_( score( self.route, f, s ) > 0 )
    self.assertAlmostEqual( 80.46, score( self.route, f, s), 1 )

if __name__ == "__main__":
  unittest.main()

