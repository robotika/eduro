#!/usr/bin/python
"""
  Read numbers from BarCodeReader (as keyboard events)
  usage:
      barcode.py <input device/file>
"""

import sys
import struct
from threading import Thread,Event,Lock

class BarcodeReader( Thread ):
  def __init__( self, deviceName='/dev/input/event0' ):
    Thread.__init__(self)
    self.f = open( deviceName, "rb")
    self.code = None
    self.index = 1000 # just offset form 0-9 numbers, to avoid accidental mismatch
    self.shouldIRun = Event()
    self.shouldIRun.set()

  def run(self):
    while self.shouldIRun.isSet():
      code = self._getCode()
      if code is not None:
        self.code = code

  def _getCode( self ):
    dummy1, dummy2, evType, evCode, evValue = struct.unpack("IIHHI", self.f.read(16) )
    if evType == 1: # keyboard event EV_KEY
      if evValue == 1: # pressed
        if 2 <= evCode <= 11:
          self.index += 1
          return ((evCode-1) % 10, self.index)  # zero is 11

  def getCode( self ):
    tmp = self.code
    self.code = None
    return tmp

  def requestStop(self):
    self.shouldIRun.clear()


if __name__ == "__main__":
  import sys
  if len(sys.argv) < 2:
    print __doc__
    sys.exit(-1)
  reader = BarcodeReader( sys.argv[1] )
  reader.start()
  while True:
    v = reader.getCode()
    if v is not None:
      print v

 
