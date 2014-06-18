"""
  Copy images clasified as balls to given directory
  usage:
     ./sortballs.py <source dir> <balls dir>
"""

import os
import sys
import shutil
import subprocess 

BALL_SIZE_LIMIT = 100


EXE = r"m:\git\eduro\c\Debug\rc.exe"
EXE_DIR = r"m:\git\cvdrone\bin\vs2008"

def isBall( path ):
  p = subprocess.Popen( EXE, shell=True, cwd=EXE_DIR, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
  p.stdin.write("file "+path+'\n')
  p.stdin.write("quit\n")
  s = [int(x) for x in p.stdout.read().split()]
  if s[0] > BALL_SIZE_LIMIT or s[1] > BALL_SIZE_LIMIT:
    print path, s[:2]
    return True
  return False

if __name__ == "__main__":
  if len(sys.argv) < 3:
    print __doc__
    sys.exit(1)
  for name in os.listdir( sys.argv[1] ):
    if name.endswith(".jpg"):
      if isBall( sys.argv[1] + os.sep + name ):
        shutil.copy( sys.argv[1] + os.sep + name, sys.argv[2] + os.sep + name )
