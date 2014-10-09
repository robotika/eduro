#!/usr/bin/python
"""
  Socket Robot Extension (both client/server side)
  usage:
     socketext.py <server/client>
"""

import sys
import socket
import traceback

from threading import Thread,Event,Lock

HOST = 'localhost'    # The remote host
#HOST = '192.168.1.6'    # The remote host
#HOST = '10.0.0.11'    # The remote host
#HOST = '192.168.1.142'    # The remote host
PORT = 50007              # The same port as used by the server 

class SocketExtension( Thread ):
  "provide robot extension for remote computing"
  def __init__( self, runAsServer ):
    Thread.__init__( self )
    self.setDaemon(True)
    self.lock = Lock()
    self.shouldIRun = Event()
    self.shouldIRun.set() 
    if runAsServer:
      self.serverSocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
      self.socket = None
      self.start()
    else: 
      self.serverSocket = None
      self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
      self.socket.connect((HOST, PORT))
    self._buffer = ""
    self._data = None

  def term( self ):
    print "CLOSING SOCKETS"
    if self.serverSocket and self.socket==None:
      print "KILLING run()"
      tmpSocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
      tmpSocket.connect((HOST, PORT))
      tmpSocket.close()
      print "DONE"
    if self.serverSocket:
      print "SHUTDOWN serverSocket"
      self.serverSocket.shutdown(socket.SHUT_RDWR)
      self.serverSocket.close()
      self.serverSocket = None
    if self.socket:
      self.socket.close()
      self.socket = None

  def run( self ):
    "wait only for the first client - delayed binding"
    self.serverSocket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    self.serverSocket.bind((HOST, PORT))
    while self.shouldIRun.isSet():
      try:
        print "serverSocket", self.serverSocket.gettimeout()
        self.serverSocket.listen(1)
        self.socket, self.addr = self.serverSocket.accept() 
        print "socket", self.socket.gettimeout()
        self.socket.settimeout(5.0)
        print "socket", self.socket.gettimeout()
        print 'Connected by', self.addr 
        while self.shouldIRun.isSet():
          self._data = self.receive() 
      except:
        print "SOCKET SERVER EXCEPTION"
        traceback.print_exc(file=sys.stderr)
        if self.socket:
          self.socket.shutdown(socket.SHUT_RDWR)
          self.socket.close()
          self.socket = None
    self.serverSocket.close()
    self.serverSocket = None

  def data( self ):
    self.lock.acquire()
    xy = self._data
    self._data = None
    self.lock.release()
    return xy 

  def extension( self, robot, id, data ):
    self.send( (id,data) )
#    if id == 'camera' and len(data)>=2 and data[1] != None:
#      if data[1] != None: 
#        self.send( ("jpgdata", open(data[0],"rb").read() ) )

  def send( self, data ):
    if self.socket:
      self.socket.send( repr( data )+'\n' )

  def receive( self ):
    "wait for new data from the socket"
    while 1:
      if '\n' in self._buffer:
        break
      data = self.socket.recv(1024)
      if not data:
        return None # or exception??
      self._buffer += data
    i = self._buffer.find('\n')
    ret = eval( self._buffer[:i] )
    self._buffer = self._buffer[i+1:]
    return ret

if __name__ == "__main__": 
  if len(sys.argv) < 2:
    print __doc__
    sys.exit(2) 
  runAsServer = (sys.argv[1]=='server')
  s = SocketExtension( runAsServer )
  if runAsServer:
    while 1:
      data = s.data()
      if data is not None:
        print data
  else:
    s.send([1,2,3])
    s.send("Ahoj!")
    s.send( ("camera", ("img123.jpg", "".join([chr(i) for i in xrange(256)]) ) ) )
    while 1:
      line = sys.stdin.readline()
      print "sending ...",
      if s is None:
        s = SocketExtension( runAsServer=False )
      try:
        s.send( line )
      except:
        print "CLIENT SOCKET EXCEPTION"
        traceback.print_exc(file=sys.stderr)
        print "terminate ..."
        s = None
      print "done."

