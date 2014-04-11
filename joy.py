#!/usr/bin/env python

"""
struct js_event {
         __u32 time;     /* event timestamp in milliseconds */
         __s16 value;    /* value */
         __u8 type;      /* event type */
         __u8 number;    /* axis/button number */
 };

	
"""

import os
import time
from ctypes import *

class js_event(Structure):
	_fields_ = [("time", c_uint32),
				("value", c_int16),
				("type", c_uint8),
				("number", c_uint8)]
	def __repr__(self):
		return "js_event<time: %d, value: %6d, type: %3d, number: %2d>" % (
				self.time, self.value, self.type, self.number)

class Joy():		
	def __init__(self, JoyDev='/dev/input/js0'):
		self.joy = os.open(JoyDev, os.O_RDONLY|os.O_NONBLOCK)
		#print self.joy
		self.a = js_event()
		self.Speed=0.
		self.Angle=0.
		self.update()

	def update(self):
		try:
			data = os.read(self.joy, sizeof(self.a))
			while len(data)>0: 
				memmove(addressof(self.a),data , sizeof(self.a))
				#print self.a
				if self.a.type==2:
					if self.a.number == 1:
						self.Speed=float(-self.a.value)/2**15
					elif self.a.number == 2:
						self.Angle=float(self.a.value)/2**15
				data = os.read(self.joy, sizeof(self.a))
		except OSError, e:
			if e.errno != 11:
				raise
				

def main():
	joy=Joy()
	while 1:
		joy.update()
		print "%6f, %6f" % (joy.Speed, joy.Angle)

if __name__ == "__main__":
	main()

