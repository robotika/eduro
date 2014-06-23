#!/usr/bin/python
"""
  Convert directory of images into video
  usage:
     ./img2video.py <directory of images> <output video file>
"""
import sys
import os 
import cv2

def img2video( images, filename ):
  writer = cv2.VideoWriter( filename, cv2.cv.CV_FOURCC('F', 'M', 'P', '4'), 10, (640,512) )
  for name in images:
    img = cv2.imread( name )
    writer.write( img )
  writer.release()


if __name__ == "__main__": 
  if len(sys.argv) < 3:
    print __doc__
    sys.exit(2)

  images = [sys.argv[1] + os.sep + path for path in os.listdir( sys.argv[1] ) if path.endswith('jpg')]
  img2video( images, sys.argv[2] )
