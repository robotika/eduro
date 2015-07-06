#!/usr/bin/python
"""
  Evaluation of Maize Field complexity (from navigation point of view)
  usage:
       ./eval_field.py <laser data/gaps>
"""
import sys
import matplotlib.pyplot as plt


def getArray( filename ):
    arr = []
    for line in open(filename):
        if "gapsize:" in line:
            arr.append( int(line.split()[-1]) )
    return arr

def draw( arr ):
    plt.plot(arr, 'o-', linewidth=2)
    plt.show()


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print __doc__
        sys.exit(2)
    draw( getArray(sys.argv[1]) )

# vim: expandtab sw=4 ts=4 

