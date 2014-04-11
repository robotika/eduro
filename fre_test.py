#!/usr/bin/env python

import unittest
import math
from fre import *



class FRETest( unittest.TestCase ):

  def testSplitScan( self ):
    self.assertEqual( splitScan( [509,   3193, 2401, 1936, 1632, 1420, 1265, 1149, 1059, 990,   1804, 1723, 1661,   2392,   3108, 1000000, 1000000, 1000000], rowsOnLeft=False ), [[1936, 1632, 1420, 1265, 1149, 1059, 990], [1804, 1723, 1661]] )


    self.assertEqual( splitScan( [1000000, 1000000, 1000000, 1000000, 1000000, 2372, 1639, 1701, 1781, 965, 1033, 1120, 1234, 1385, 1591, 1888, 2342, 3114], rowsOnLeft=True), [[1639, 1701, 1781], [965, 1033, 1120, 1234, 1385, 1591, 1888]])


  def testPerpDist( self ):
    self.assertAlmostEqual( perpDist(1.41, 1.41, math.radians(90)), 1.0, 2 )

if __name__ == "__main__":
  unittest.main()


