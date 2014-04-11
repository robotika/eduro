#!/usr/bin/env python
"""
 RayTrace for MonteCarloLocalisation
"""
# instead of line with start/end it could be rewritten for simply ((x1,y1),(x2,y2))

import math
from line import distance

def normalizeAnglePIPI( angle ):
  "normalize only within 2PI error"
  if angle < -math.pi:
    angle += 2*math.pi
  if angle > math.pi:
    angle -= 2*math.pi
  return angle


def combinedPose( robotPose, sensorPose ):
  return (
    robotPose[0] + sensorPose[0] * math.cos( robotPose[2] ) - sensorPose[1] * math.sin( robotPose[2] ),
    robotPose[1] + sensorPose[0] * math.sin( robotPose[2] ) + sensorPose[1] * math.cos( robotPose[2] ),
    normalizeAnglePIPI( robotPose[2] + sensorPose[2] ) )


def rayTraceSingleLine( pose, wallStart, wallEnd, maxRadius ):
  a = wallStart[1] - wallEnd[1]
  b = wallEnd[0] - wallStart[0]
  c = - a * wallStart[0] - b * wallStart[1]

  frac = a * math.cos( pose[2] ) + b * math.sin( pose[2] )
  if math.fabs( frac ) < 0.0001:
    return maxRadius

  t = -(a * pose[0] + b * pose[1] + c)/frac;
  if t < 0:
    return maxRadius

  # check that crossing belongs to the wall and is not outside
  crossing = ( pose[0] + t * math.cos( pose[2] ), pose[1] + t * math.sin( pose[2] ) )
  if distance( crossing, wallStart ) + distance( crossing, wallEnd ) > distance( wallStart, wallEnd ) + 0.01:
    return maxRadius
  return t < maxRadius and t or maxRadius


def rayTrace( pose, obstacles, maxRadius = 1000 ):
  "return distance to nearest obstacle"
  dist = maxRadius
  for obj in obstacles:
    a = obj[0]
    for b in obj[1:]:
      dist = rayTraceSingleLine( pose, a, b, dist )
      a = b
  return dist

