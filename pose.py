#!/usr/bin/env python
"""
 Simple pose operations
"""

import math

def normalizeAnglePIPI( angle ):
  while angle < -math.pi:
    angle += 2*math.pi
  while angle > math.pi:
    angle -= 2*math.pi
  return angle


def combinedPose( robotPose, sensorPose ):
  return (
    robotPose[0] + sensorPose[0] * math.cos( robotPose[2] ) - sensorPose[1] * math.sin( robotPose[2] ),
    robotPose[1] + sensorPose[0] * math.sin( robotPose[2] ) + sensorPose[1] * math.cos( robotPose[2] ),
    normalizeAnglePIPI( robotPose[2] + sensorPose[2] ) )

def inversePose( pose ):
  "return inverse pose to (0,0,0)"
  x,y,a = pose
  return -x*math.cos(a)-y*math.sin(a), x*math.sin(a)-y*math.cos(a), -a

