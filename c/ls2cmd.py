import os
import sys

for name in os.listdir( sys.argv[1] ):
  if name.endswith(".jpg"):
    print "file", sys.argv[1] + os.sep + name
