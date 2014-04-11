#!/usr/bin/env python

import math

class SimpleOdometry():
  def __init__( self, pose = (0,0,0) ):
    self.x = pose[0]
    self.y = pose[1]
    self.heading = pose[2]
  def setPxPa( self, dist, angle ):
    # advance robot by given distance and angle
    if math.fabs(dist) > 1.0:
      print dist, angle
    if angle == 0.:
      # Straight movement - a special case
      self.x += dist * math.cos(self.heading)
      self.y += dist * math.sin(self.heading)
      #Not needed: self.heading += angle
    else:
      # Arc
      r = dist / angle
      self.x += -r * math.sin(self.heading) + r * math.sin(self.heading + angle)
      self.y += +r * math.cos(self.heading) - r * math.cos(self.heading + angle)
      self.heading += angle # not normalized

  def pose( self ):
    return (self.x, self.y, self.heading)

  def setPose( self, pose ):
    (self.x, self.y, self.heading) = pose

  def updateCompass( self, compassData ):
    pass

  def updateGPS( self, gpsData ):
    pass

  def updateSharps( self, sharps ):
    pass

class KalmanFilter( SimpleOdometry ):
  """
    Simple KF version without correlations in x, y, heading variables.
    http://en.wikipedia.org/wiki/Normal_distribution
    f = a * exp( -(x-mean)**2/2*sigma**2 )
    - sigma is called standard deviation
    - sigma**2 is the variance of the distribution

About 68% of values drawn from a normal distribution are within one standard
deviation sigma > 0 away from the mean sigma; about 95% of the values are
within two standard deviations and about 99.7% lie within three standard
deviations.

varX ... variance of the X distribution (None = undefined)

    kf.conv needs to be set to a route.Convertor instance in order to use the GPS
  """

  #TODO: Configurable parameters (e.g. accuracy of odometry, compass accuracy, ...)
  #TODO: Calibrate the parameters
  #TODO: A proper EKF, UKF or even GraphMCL
  def __init__( self, pose = (0,0,0), compassOffset = math.radians(90) ):
    SimpleOdometry.__init__( self, pose )
    self.varXY = None # unknown position
    self.varHeading = None # unknown heading
    self.compassOffset = compassOffset
    self.conv = None

  def setPxPa( self, dist, angle ):
    SimpleOdometry.setPxPa( self, dist, angle )
    if self.varXY != None:
      self.varXY = self.varXY + (dist * 0.1)**2 # 10% error
    if self.varHeading != None:
      self.varHeading = self.varHeading + (angle * 0.1)**2

  def updateCompass( self, compassData ):
    varCompass = 0.1**2
    angle = self.compassOffset - math.radians( compassData/10.0 ) # there could be some other North offset
    if self.varHeading is None:
      self.heading = angle
      self.varHeading = varCompass
    else:
      totalVar = self.varHeading + varCompass
      c = (math.cos(self.heading) * varCompass + math.cos(angle) * self.varHeading) / totalVar
      s = (math.sin(self.heading) * varCompass + math.sin(angle) * self.varHeading) / totalVar
      self.heading = math.atan2( s, c )
      self.varHeading = (varCompass * self.varHeading) / totalVar # Equivalent to 1/(1/self.varHeading + 1/varCompass)

  def updateGPS( self, gpsData ):
    if self.conv is None:
      return # cannot convert GPS coordinates to the local metric system

    lat,lon,sat,hdop = gpsData
    devGPS = 4.0 * hdop # http://users.erols.com/dlwilson/gpshdop.htm
    varGPS = devGPS**2
    x, y = self.conv((lon, lat))
    if self.varXY is None:
      self.x = x
      self.y = y
      self.varXY = varGPS
    else:
      K = self.varXY/(self.varXY + varGPS)
      self.x += K * (x-self.x)
      self.y += K * (y-self.y)
      self.varXY *= (1.0 - K)   # see http://robotika.cz/guide/filtering/cs
                                # it is the same as for compass

class Multilocalisation():
  '''
  Multilocalisation binds multiple localizations into a single one.

  This allows multiple localizations (e.g. a local and a global one)
  to be attached to the robot at the same time and being updated at the same
  time.
  '''
  def __init__( self, basicLocalizations, primaryLocalization=None ):
    '''
    Parameters:
      basicLocalizations  ... List of basicLocalizations bound together
                              by this Multilocalisation.
      primaryLocalization ... A localization which is used to provide
                              pose in the pose() function. If set to None,
                              the first of basicLocalizations is used.
    '''
    self.basicLocalizations = basicLocalizations
    if primaryLocalization is None:
      primaryLocalization = basicLocalizations[0]
    self.primaryLocalization = primaryLocalization

  def setPxPa( self, dist, angle ):
    for loc in self.basicLocalizations:
      loc.setPxPa(dist, angle)

  def pose( self ):
    return self.primaryLocalization.pose()

  def setPose( self, pose ):
    for loc in self.basicLocalizations:
      loc.setPose(pose)

  def updateCompass( self, compassData ):
    for loc in self.basicLocalizations:
      loc.updateCompass(compassData)

  def updateGPS( self, gpsData ):
    for loc in self.basicLocalizations:
      loc.updateGPS(gpsData)

  def updateSharps( self, sharps ):
    for loc in self.basicLocalizations:
      loc.updateSharps(sharps)


