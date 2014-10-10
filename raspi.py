#!/usr/bin/python
"""
  XMLRPC client to RaspberryPi "camera processor"
  usage:
      raspi.py url
"""

import sys
import struct
from threading import Thread,Event,Lock
import xmlrpclib

class RasPi( Thread ):
    def __init__( self, url='http://192.168.1.1:8000' ):
        Thread.__init__(self)
        self.url = url
        self.server = None
        self.data = None
        self.shouldIRun = Event()
        self.shouldIRun.set()

    def run(self):
        self.server = xmlrpclib.ServerProxy( self. url )
        self.server.init()
        while self.shouldIRun.isSet():
            self.data = self.server.step()
        self.server.term()

    def getData( self ):
        tmp = self.data
        self.data = None
        return tmp

    def requestStop(self):
        self.shouldIRun.clear()

if __name__ == "__main__":
    import sys
    if len(sys.argv) < 2:
        print __doc__
        sys.exit(-1)
    raspi = RasPi( sys.argv[1] )
    raspi.start()
    while True:
        v = reader.getData()
        if v is not None:
            print v
            break

#-------------------------------------------------------------------
# vim: expandtab sw=4 ts=4 
 
