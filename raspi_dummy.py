#!/usr/bin/python
"""
  !!!DUMMY!! The main program of the comteticion Sick Robot Day 2014
"""

import sys

from SimpleXMLRPCServer import SimpleXMLRPCServer
from SimpleXMLRPCServer import SimpleXMLRPCRequestHandler

# Restrict to a particular path.
class RequestHandler(SimpleXMLRPCRequestHandler):
    rpc_paths = ('/RPC2',)



class MyFuncs:
    def init( self ):
        return 1

    def step( self ):
        detected = [] #['ahoj']
        return detected

    def term( self ):
        return 1

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print __doc__
        sys.exit()
    print "STARTED"
    server = SimpleXMLRPCServer(("localhost", 8013),
                            requestHandler=RequestHandler)
    server.register_introspection_functions()
    server.register_instance(MyFuncs())
    server.serve_forever()

#-------------------------------------------------------------------
# vim: expandtab sw=4 ts=4 

