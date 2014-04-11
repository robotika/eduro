#!/usr/bin/python
"""
  Eduro Robot
"""

from robot import Robot

shortSharpTab = (
      ( 32767.0, 0.0 ),
      ( 18368.0, 0.060 ),
      ( 13920.0, 0.107 ),
      ( 9984.0, 0.157 ),
      ( 8032.0, 0.207 ),
      ( 6784.0, 0.257 ),
      ( 5696.0, 0.307 ),
      ( 5120.0, 0.357 ),
      ( 4736.0, 0.407 ),
      ( 4384.0, 0.457 ),
      ( 4512.0, 0.507 ),
      ( 4224.0, 0.556 ),
      ( 3904.0, 0.607 ),
      ( 3648.0, 0.656 ),
      ( 3392.0, 0.706 ),
      ( 3232.0, 0.756 ),
      ( 0.0, 1.0 ))

longSharpLeftTab = (
      ( 32767.0, 0.0 ),
      ( 16864.0, 0.1 ),
      ( 16544.0, 0.2 ),
      ( 13440.0, 0.3 ),
      ( 10080.0, 0.4 ),
      ( 8288.0, 0.5 ),
      ( 6976.0, 0.6 ),
      ( 6080.0, 0.7 ),
      ( 5504.0, 0.8 ),
      ( 4960.0, 0.9 ),
      ( 4864.0, 1.0 ),
      ( 0.0, 2.0 ))

longSharpRightTab = (
      ( 32767.0, 0.0 ),
      ( 16448.0, 0.1 ),
      ( 15680.0, 0.2 ),
      ( 12000.0, 0.3 ),
      ( 9152.0, 0.4 ),
      ( 7360.0, 0.5 ),
      ( 5984.0, 0.6 ),
      ( 5056.0, 0.7 ),
      ( 4448.0, 0.8 ),
      ( 3776.0, 0.9 ),
      ( 3296.0, 1.0 ),
      ( 0.0, 2.0 ))

def sharp2dist( val, tab ):
  for i in range(len(tab)):
    if val > tab[i][0]:
      break
  assert( i > 0 )
  return tab[i-1][1] + (val - tab[i-1][0])/(tab[i][0]-tab[i-1][0]) * (tab[i][1]-tab[i-1][1])


class Eduro( Robot ):
  def __init__( self, can=None, maxAcc = 1.0, replyLog=None ):
    Robot.__init__( self, can, maxAcc, replyLog ) 

  def updateSharps( self, id, data ):
    if id == 0x183:
      assert( len(data) == 8 )
      s0 = sharp2dist( data[0] + 256*data[1], longSharpLeftTab ) # diff orientation
      s1 = sharp2dist( data[2] + 256*data[3], longSharpRightTab )
      s2 = sharp2dist( data[4] + 256*data[5], shortSharpTab )
      self.sharp = (s0, s1, s2)
      if self.localisation:
        self.localisation.updateSharps( self.sharp )


class EmergencyStopException( Exception ):
  pass

def emergencyStopExtension( robot, id, data ):
  if robot.emergencyStop:
    raise EmergencyStopException()

