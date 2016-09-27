# vim: set fileencoding=utf-8 et sts=4 sw=4:
from __future__ import absolute_import, division, print_function, unicode_literals
from math import cos, sin
from collections import namedtuple
Position = namedtuple("position", ("x", "y", "a"))

def transponse_raw(relx, rely, x, y, a):
    return x+cos(a)*relx-sin(a)*rely, y+sin(a)*relx+cos(a)*rely

def transponse(pos, offset):
    if type(pos) != Position:
        pos=Position(*pos)
    if type(offset) != Position:
        offset=Position(*offset)
    x,y=transponse_raw(offset.x, offset.y, pos.x, pos.y, pos.a)
    return Position(x, y, offset.a+pos.a)
