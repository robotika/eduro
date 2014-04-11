#!/usr/bin/python
"""
  COMPASS
  usage:
     ./compass.py <log file to check>
"""

import sys
import math
from can import CAN, ReplyLog 

from can import parseFileG

def sint16( data ):
  ret = data[1]*256+data[0]
  if ret > 0x8000:
    ret = ret-0x10000
  return ret

def parseCompass( filename, watchModules = [] ):
  for io, id, data in parseFileG( filename, watchModules ):
    if io == 0:
      pass
    elif io == 1:
      if id == 0x187: # 3D compass
        print "c\t%d\t%d\t%d" % (sint16(data[0:2]), sint16(data[2:4]), sint16(data[4:6]) )
      if id == 0x387: # 3D accelerometer
        print "a\t%d\t%d\t%d" % (sint16(data[0:2]), sint16(data[2:4]), sint16(data[4:6]) )
      if id == 0x487: # 3D raw compass data
        print "r\t%d\t%d\t%d" % (sint16(data[0:2]), sint16(data[2:4]), sint16(data[4:6]) )
      if id == 0x181:# or id == 0x182:
        e = data[0]|(data[1]<<8)|(data[2]<<16)|(data[3]<<24) 
        print "e\t%x\t%d" % (id, e)



def analyseCompass( compassRaw, compassAcc ):
  # horizontal plane is given by ax+by+cz+d=0, where (a,b,c)=compassAcc and d=0
  
  # incination in Prague is 66deg, declination is 11deg
  inc = math.radians(66.0)
  vecB = (math.cos(inc), 0.0, math.sin(inc)) # handle declination afterwards

#  compassOffset = (-150, 550, 2250)
  compassOffset = (-221, -108, 2250) # with hand


  # vector multiplication
  # C = A x B = (AyBz - ByAz, AzBx-BzAx, AxBy-BxAy)
  A = compassAcc # in reality z-coordinate
  B = (compassRaw[0]-compassOffset[0], compassRaw[1]-compassOffset[1], compassRaw[2]-compassOffset[2])
  C = (A[1]*B[2]-B[1]*A[2], A[2]*B[0]-B[2]*A[0], A[0]*B[1]-B[0]*A[1])
  return int(10*math.degrees(math.atan2(C[0],C[1])))

def analyseCompassEx( compass, compassRaw, compassAcc ):
  return compass/10, analyseCompass( compassRaw, (0,0,1) )/10,  (analyseCompass( compassRaw, compassAcc ) - analyseCompass( compassRaw, (0,0,1) ))/10

if __name__ == "__main__":
  import sys
  if len(sys.argv) > 1:
    parseCompass( sys.argv[1], [int(x) for x in sys.argv[2:]] )
  else:
    print __doc__
    # measurement in approx NE direction
    print analyseCompassEx( 701, (1208, -2574, 6313), (509, -404, 8490) )
    print analyseCompassEx( 1093, (1101, -2388, 6437),(2488, -319, 8602)  ) # game1
    print analyseCompassEx( 215, (2931, -1151, 6376), (-292, 126, 8364) )
    print analyseCompassEx( 1263, (-643, -2430, 6473), (1695, 152, 8607) )

#    print analyseCompass( 459, (-150+1000,0+550,6000), (0,0,1) ) # dummy
#    print analyseCompass( 459, (2121,-1681,6069), (341,376,8547) ) # horizontal
#    print analyseCompass( 428, (2923,-1631,5459), (1478,284,8494) ) # tilt forward
#    print analyseCompass( 417, (3621,-1635,4554), (2613,399,8128) ) # bigger tilt forward
#    print analyseCompass( 470, (1742,-568,6929), (-129,1767,8439) ) # tilt to right
#    print analyseCompass( 411, (1474,-2466,5774), (-721,-1050,8399) ) # tilt to left

 
