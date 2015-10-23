#!/usr/bin/python
"""
  Eduro (Maxi) Proxy server/client
  usage:
       ./eduroproxy.py <server/client> <host> <port>
"""
import sys
import socket
import datetime
import struct
import time

# TODO log record & replay
# format binary, json, ROS like, ??? (remember old python on Eduro)

# general, or based on existin inputs (CAN, LASER, GPS, CAMERA)
# who will do the attachemnt??
# i.e. "control messages"???

# multiple "clients" - NO!
# ... but online viewer + some online navigation???
# ... different topics and registrations?


SOCKET_TIMEOUT = 0.0


def proxyServer( ip ):
    host, port = ip
    serverSocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    serverSocket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    serverSocket.bind((host, port))
    print "Waiting ..."
    serverSocket.listen(1)
    soc, addr = serverSocket.accept() 
    print 'Connected by', addr
    data = soc.recv(1024) # TODO properly load and parse/check
    print data
    print "LEN", len(data)
    header = "Ahoj" # hack
    soc.send( header )
    return soc


def proxyClient( ip ):
    host, port = ip
    soc = socket.socket(socket.AF_INET, socket.SOCK_STREAM) # TCP
    soc.connect( (host,port) )
#    soc.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
#    soc.setblocking(0)
#    soc.settimeout( SOCKET_TIMEOUT )
    header = "Nazdar!" # hack2
    soc.send( header )
    print soc.recv(1024) # hack3 - win exception "Errno 10035"
    return soc


if __name__ == "__main__":
    if len(sys.argv) < 4:
        print __doc__
        sys.exit(2)
    assert sys.argv[1] in ["server", "client"], sys.argv[1]
    host, port = sys.argv[2], int(sys.argv[3])
    if sys.argv[1] == "server":
        proxyServer( (host, port) )
    else:
        proxyClient( (host, port) )
    
# vim: expandtab sw=4 ts=4 

