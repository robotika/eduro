#!/usr/bin/python

def starter( can, configFn=None ):
  can.sendPreoperationMode()
  print "Waiting for release of STOP button"
  while 1:
    id, data = can.readPacket()

    if id == 0x8A:
      print data
      assert( len(data) == 8 )
      emergencyStop = (data[0:2] == [0,0x10]) and (data [3:] == [0,0,0,0,0])
      print "EmergencyStop:", emergencyStop
      if not emergencyStop:
        break
  
  return can.resetModules( configFn=configFn )

if __name__ == "__main__": 
  from can import CAN
  can = CAN()
  print "Started modules: ", starter( can )
  del can
  print "Starter terminated"

