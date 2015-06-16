"""
  Recalculate camera logfile with new binary code
  usage:
     ./recalcam.py <camera log file>
"""

import os
import sys
import shutil
import subprocess 

BALL_SIZE_LIMIT_MIN = 100
BALL_SIZE_LIMIT_MAX = 150


EXE = r"m:\git\eduro\c\Debug\rc.exe"
EXE_DIR = r"m:\git\cvdrone\bin\vs2008"

def processFile( path ):
    p = subprocess.Popen( EXE, shell=True, cwd=EXE_DIR, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    p.stdin.write("file "+path+'\n')
    p.stdin.write("quit\n")
    #s = [int(x) for x in p.stdout.read().split()]
    return p.stdout.read()

def recalculateCameraLog( filename ):
    f = open( filename )
    while True:
        print f.readline().strip()
        line = f.readline().strip()
        if len(line) == 0:
            break
        oldResult, oldFilename = eval( line )
        if oldResult is not None:
            path = os.path.join(os.path.split( filename )[0], os.path.split( oldFilename )[1])
            print (processFile( path ).replace('\r',''), oldFilename)
        else:
            print (oldResult, oldFilename)


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print __doc__
        sys.exit(1)
    recalculateCameraLog( sys.argv[1] )

# vim: expandtab sw=4 ts=4 

