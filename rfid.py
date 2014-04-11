#!/usr/bin/python
"""
  RFID receiver over Ethernet (default 192.168.1.7:10001)
  usage:
      rfid.py <log file>
"""

import sys
import socket
import time
from threading import Thread,Event,Lock

HOST = '192.168.1.7'    # The remote host
PORT = 10001            # The same port as used by the server
ETX = '\n'

class RFID( Thread ):
  def __init__( self ):
    Thread.__init__(self) 
    self.setDaemon(True)
    self.lock = Lock()
    self.shouldIRun = Event()
    self.shouldIRun.set()
    self._data = None 
    self._buffer = ""
    self.socket = None

  def run(self):
    while self.shouldIRun.isSet():
      self._data = self.receive()

  def connect( self ):
    self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    while True:
      try:
        self.socket.connect((HOST, PORT))
        break
      except socket.error:
        print "Socket error" # repeat
        time.sleep(5.0)

  def receive( self ):
    data = self._buffer
    while True:
      pos = data.find(ETX)
      if pos >= 0:
        break
      if self.socket == None:
        self.connect()
      try:
        data = self.socket.recv(1024)
      except socket.error:
        print "Socket receive error"
        self.socket = None
        data = ""
      self._buffer += data

    pos = self._buffer.find(ETX)
    assert( pos >= 0 )
    data = self._buffer[:pos]
    self._buffer = self._buffer[pos+1:]
    return data
 
  def data( self ):
    self.lock.acquire()
    xy = self._data
    self.lock.release()
    return xy

  def requestStop( self ):
    self.shouldIRun.clear()

  def __del__( self ):
    if self.socket:
      self.socket.close()


def testRFID( logfile ):
  rfid = RFID()
  f = open( logfile, "wb" )
  while True:
    f.write( rfid.receive() + '\n' )
    f.flush()


def testThreadedRFID( logfile, num = 30 ):
  rfid = RFID()
  rfid.start()
  f = open( logfile, "wb" )
  i = 0
  prev = None
  while True:
    data = rfid.data()
    if data != prev:
      print data
      f.write( data + '\n' )
      f.flush()
      i += 1
      if i >= num:
        break
    prev = data
  rfid.requestStop()
  rfid.join() 


if __name__ == "__main__":
  import sys
  if len(sys.argv) < 2:
    print __doc__
    sys.exit(-1)
  testThreadedRFID( sys.argv[1] )

 
