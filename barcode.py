#!/usr/bin/python
"""
  Read numbers from BarCodeReader (as keyboard events)
  usage:
      barcode.py <input device/file>
"""

import sys
import struct

class BarCodeReader:
  def __init__( self, deviceName='/dev/input/event0' ):
    self.f = open( deviceName, "rb")

  def getCode( self ):
    dummy1, dummy2, evType, evCode, evValue = struct.unpack("IIHHI", self.f.read(16) )
    if evType == 1: # keyboard event EV_KEY
      if evValue == 1: # pressed
        if 2 <= evCode <= 11:
          return (evCode-1) % 10  # zero is 11

if __name__ == "__main__":
  import sys
  if len(sys.argv) < 2:
    print __doc__
    sys.exit(-1)
  reader = BarCodeReader( sys.argv[1] )
  while True:
    v = reader.getCode()
    if v is not None:
      print v

 
