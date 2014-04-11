#!/usr/bin/env python

import unittest
import math
from ray_trace import *

class RayTraceTest( unittest.TestCase ):

  def testLine( self ):
    walls = [ ((10,-2), (10,2)) ]
    self.assertAlmostEqual( rayTrace( (0,0,0), walls, 150 ), 10.0, 5)
    self.assertAlmostEqual( rayTrace( (0,0,math.radians(90)), walls, 150 ), 150.0, 5)
    self.assertAlmostEqual( rayTrace( (0,0,math.radians(45)), walls, 150 ), 150.0, 5)

  def testTwoWalls( self ):
    obstacles = [ ((10,-2), (10,2)), ((5,-2), (5,2)) ]
    self.assertAlmostEqual( rayTrace( (0,0,0), obstacles, 150 ), 5.0, 5)

  def testCombinedPose( self ):
    self.assertEqual( combinedPose( (0,0,0), (0,0,0) ), (0,0,0) )
    self.assertEqual( combinedPose( (0,0,0), (0.1,0.2,0) ), (0.1,0.2,0) )
    self.assertEqual( combinedPose( (10,5,math.radians(90)), (0.1,0.2,0) ), (10-0.2,5+0.1,math.radians(90)) )

if __name__ == "__main__":
  unittest.main()


