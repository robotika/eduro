#!/usr/bin/python

import unittest
from sickday2014 import *

class SickdayTest( unittest.TestCase ):

  def testComputeLoadManeuver( self ):
    self.assertEqual( computeLoadManeuver( None, None, None ), (math.radians(180), 0.0) )

    # real data when approaching with wall on the right side ... motion should be negative?!
    self.assertEqual( computeLoadManeuver( 0.445, 0.43, 0.409), (-2.919790583223523, -0.10080306540518491) )

    # Tomas's conversion
    A = 0.409
    B = 0.43
    Rs = 0.15
    Rm = 0.35
    L = 0.25
    k = 1 + Rs/B
    C = math.sqrt( B*B - A*A )
    D = math.sqrt( (k*A - L)**2 + (k*C)**2 )
    X = D - Rm
    alpha = math.asin( L*C/(D*B) )
    self.assertAlmostEqual( alpha, -2.919790583223523+math.radians(180) )

    # straight approach ... I am not sure about the 5cm backup
    self.assertEqual( computeLoadManeuver( 0.41, 0.4, 0.39999999), (-3.1414063145985467, -0.05000001145833316) )

if __name__ == "__main__":
  unittest.main()

#-------------------------------------------------------------------
# vim: expandtab sw=4 ts=4  
