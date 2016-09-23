"""
  Universal template for competitions
  usage:
      <competition name> <config file> [--file <log file> [run number 1..N] [F|FF]|--simul]
"""
#try: # Use Cython, if available
#  import pyximport; pyximport.install(pyimport=True)
#except:
#  pass

try: # Use Psyco, if available.
  import psyco
  psyco.full()
except ImportError:
  pass

import sys
import os

from can import CAN, ReplyLog, ReplyLogInputsOnly, LogEnd
import starter
import viewlog
from viewlog import viewLogExtension, viewCompassExtension, viewPoseExtension, ViewCameraExtension

#TODO: Lots of work.
from eduro import EmergencyStopException, emergencyStopExtension
from laseranalysis import LaserViewer
from simulator import Simulator
from camera import timeName

def __run(robot, task, configFilename, verbose=False):
  try:
    contest = task( robot, configFilename, verbose=verbose )
    contest()
  except EmergencyStopException, e:
    print "EmergencyStopException"
  except KeyboardInterrupt, e:
    print "KeyboardInterrupt"
    print e
    return False
  except LogEnd, e:
    print "End of log"
    return False
  return True

def launch(cmd_args, robotFactory, task, configFn = None, canArgs={}):
  '''
  Launches a robot to the given file.

  Parameters:
  cmd_args     ... List of parameters in the "configFile [--file logfile [F|FF]]" form.
  robotFactory ... Function (or class) which can be called with the following named
                   parameters and creates an instance of the robot:
		   robot = robotFactory(can=..., replyLog=...)
  task         ... Factory which called as "t = task(robot, configFileName, verbose=...)"
                   creates a runable instance. Ie. "t()" runs the show. 
  configFn     ... A method called in the robot's preconfigration mode.
  canArgs      ... Named arguments passed down to the CAN.
  '''
  if len(cmd_args) < 2:
    print __doc__
    sys.exit(-1)
  configFilename = cmd_args[1]
  logName = None
  simulation = None
  metaLog = None

  if len(cmd_args) > 2:
    if cmd_args[2]=='--file':
      logName = cmd_args[3]
      if "meta" in logName:
        runNumber = 1
        try:
          if 'F' == cmd_args[-1] or 'FF' == cmd_args[-1]:
            runNumber = int(cmd_args[-2])
          else:
            runNumber = int(cmd_args[-1])
        except:
          pass
        print "METALOG", logName, runNumber
        metaLog = open(logName)
        for line in metaLog:
          if line.startswith("CANlog:"):
            runNumber -= 1
            if runNumber <= 0:
              logName = logName[:logName.find("meta")]+line.split()[1].split("/")[-1]
              break
      if len(cmd_args)> 4 and cmd_args[-1]=='FF':
        can = CAN( ReplyLogInputsOnly( logName ), skipInit = True )
      else:
        assertWrite = not (len(cmd_args) > 4 and cmd_args[-1]=='F') 
        can = CAN( ReplyLog( logName, assertWrite=assertWrite ), skipInit = True ) 
    else:
      assert( cmd_args[2]=='--simul' )
      simulation = True # TODO
  else:
    metaLog = open( timeName("logs/meta", "log"), "w" )
    metaLog.write( str(cmd_args) + "\n" )
    metaLog.write( "configFilename: '" + str(configFilename) + "'\n" )
    if configFilename:
      try:
        metaLog.write( "------------------\n" )
        metaLog.write( open(configFilename).read() )
        metaLog.write( "------------------\n" )
      except:
        metaLog.write( "Load FAILED\n" )
    metaLog.flush()

    can = CAN(**canArgs)

  isFromLog = logName is not None

  while 1:
    if not isFromLog and not simulation:
      can.relog('logs/s')
      print starter.starter( can, configFn=configFn )
      metaLog.write( "CANlog:\t" + str(can.relog('logs/r')) + "\n" )
      metaLog.flush()

    if simulation:
      robot = Simulator()
    else:
      robot = robotFactory(can=can, replyLog=logName, metaLog=metaLog)

    if isFromLog or simulation:
      viewlog.viewLogFile = open( "view.log", "w" )
      robot.addExtension( viewLogExtension )
#      robot.addExtension( viewCompassExtension )
      robot.addExtension( viewPoseExtension )
      laserViewer = LaserViewer( robot, distanceLimit = 20.0, sideLimit = 10 )
    if isFromLog:
      view = ViewCameraExtension( os.path.dirname( sys.argv[3] ) )
      robot.addExtension( view.viewCameraExtension )         
  
    if not __run( robot, task, configFilename, verbose=(isFromLog or simulation) ) or isFromLog or simulation:
      break
    del robot # robot has switch to preoperation ... probably not a good idea

