#!/usr/bin/env python

import unittest
import math
from pose import *

class PoseTest( unittest.TestCase ):
  def poseEqual( self, pose1, pose2 ):
    presicion = 5
    self.assertAlmostEqual( pose1[0], pose2[0], presicion )
    self.assertAlmostEqual( pose1[1], pose2[1], presicion )
    self.assertAlmostEqual( normalizeAnglePIPI(pose1[2]-pose2[2]),0, presicion ) 

  def testCombinedPose( self ):
    self.assertEqual( combinedPose( (0,0,0), (0,0,0) ), (0,0,0) )
    self.assertEqual( combinedPose( (0,0,0), (0.1,0.2,0) ), (0.1,0.2,0) )
    self.assertEqual( combinedPose( (10,5,math.radians(90)), (0.1,0.2,0) ), (10-0.2,5+0.1,math.radians(90)) )

  def testInversePose( self ):
    self.assertEqual( inversePose( (0,0,0) ), (0,0,0) )
    self.assertEqual( inversePose( (0.2,0,0) ), (-0.2,0,0) )
    self.poseEqual( inversePose( (1,2,math.radians(90)) ), (-2,1,math.radians(-90)) )

if __name__ == "__main__":
  unittest.main()


