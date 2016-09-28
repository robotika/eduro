# vim: set fileencoding=utf-8 et sts=4 sw=4:
from __future__ import absolute_import, division, print_function, unicode_literals
import sys
import os
from collections import namedtuple
from struct import unpack, calcsize
sys.path.append(os.path.dirname(os.path.dirname(__file__)))
from eduromaxi import EduroMaxi
from localisation import KalmanFilter
from common import transponse, Position

scan = namedtuple('scan',('id', 'RSSI', 'power'))

def getlogs(fn):
    """read metalog as follows:
['./sickday2016.py', 'experiment-with-velodyne']
configFilename: 'experiment-with-velodyne'
------------------
Load FAILED
CANlog:	logs/r160913_174309.log
CAMERALOG:	logs/src_camera_160913_174309.log
RFU620LOG:	logs/src_rfu620_160913_174309.log
"""

    path = os.path.split(fn)[0]
    with open(fn, 'r') as f:
        run=[]
        position=(0,0,0)
        runnumber=0
        for line in f:
            if 'position:' in line.lower():
                position=eval(line.split(':')[-1])
            if 'log:' in line.lower():
                type,name=line.split(':')
                if type == "CANlog":
                    if run:
                        yield runnumber, position, run
                    run = []
                    runnumber+=1
                    position=(0,0,0)
                run.append( (type,os.path.join(path, name.strip().split(r'/')[-1])))
        if run:
            yield runnumber, position, run
        else:
            raise Exception("No runs detected in %s"%fn)

def textlogreader(fn):
    with open(fn, 'r') as f:
        tickline=True
        for line in f:
            if tickline:
                ticks=int(line)
                #ugly hack for bad counting
                if ticks:
                    ticks-=1
                for i in range(ticks):
                    yield None
            else:
                yield eval(line)
            tickline=not tickline
        while True:
            yield None

def MetaLogReader(fn):
    for runnumber, position, run in getlogs(fn):
        print('*'*60)
        readers=[]
        names=[]
        for type, fn in run:
            names.append(type)
            if type=="CANlog":
                readers.append(MsgGrouper(MsgDummyInterpreter(Msgparser(fn)), filter_origin=["can"]))
            else:
                readers.append(textlogreader(fn))
        for msgs in zip(*readers):
            yield runnumber, position, dict(zip(names, msgs))



def BinaryLogReader(fn):
    with open(fn, 'rb') as f:
        msg = []
        lasttype = -1
        while True:
            try:
                type, byte = f.read(2)
            except ValueError:
                return
            if type != lasttype:
                if msg:
                    yield lasttype, str().join(msg)
                msg = []
                lasttype = type
            msg.append(byte)


def parseheader(header):
    rtr = (ord(header[1]) >> 4) & 0x1
    len = (ord(header[1])) & 0x0f
    id = ((ord(header[0])) << 3) | ((ord(header[1])) >> 5)
    return rtr, len, id


def Msgparser(fn, timeperiod = 0.2, timesyncpacket=0x80):
    time = 0.0
    for type, msg in BinaryLogReader(fn):
        while msg:
            rtr, datalen, id = parseheader(msg)
            if id == timesyncpacket:  # 0x281:
                time += timeperiod
            yield time, type, id, msg[2: 2 + datalen]
            msg = msg[2 + datalen:]

msgnt = namedtuple('msgnt', ('name', 'parameters', 'unpackstr'))


class Msg:
    def __init__(self, name):
        self.name = name


msg_definition = {
    0x281: msgnt(name="throtleinfo", parameters=('position',), unpackstr=r'<H'),
    0x201: msgnt(name="motorcontrol", parameters=('direction',), unpackstr=r'<B'),
    0x284: msgnt(name="encoderinfo", parameters=('left', 'right'), unpackstr=r'<HH'),
    0x182: msgnt(name="wheelangle", parameters=('angle',), unpackstr=r'<h'),
    0x28A: msgnt(name="buttons", parameters=('state',), unpackstr=r'<H'), #self.startCableIn = (state & 0x0100 ) != 0 self.switchBlueSelected = (mask & 0x0200 ) == 0 self.buttonPause = not self.switchBlueSelected
    }

def MsgDummyInterpreter(generator):
    for time, type, id, data in generator:
        msg_origin = {'\x01': 'can', '\x00': 'pc'}[type]
        yield time, msg_origin, id, [ord(x) for x in data]

def MsgInterpreter(generator):
    for time, type, id, data in generator:
        msg_origin = {'\x01': 'can', '\x00': 'pc'}[type]
        #print(hex(id), msg_origin)
        if id in msg_definition:
            mdef = msg_definition[id]
            msg = Msg(mdef.name)
            for par, value in zip(mdef.parameters, unpack(mdef.unpackstr, data[:calcsize(mdef.unpackstr)])):
                msg.__dict__[par] = value
            yield time, msg_origin, id, msg

def MsgGrouper(generator, filter_origin=None):
    lasttime, msg_origin, id, msg = generator.next()
    group=[]
    if not filter_origin or msg_origin in filter_origin:
        group.append((id,msg))
    for time, msg_origin, id, msg in generator:
        #print(time,msg_origin)
        if time <> lasttime:
            yield lasttime, group
            group=[]
            lasttime=time
        if not filter_origin or msg_origin in filter_origin:
            group.append((id,msg))
    yield lasttime, group

class DummyCan:
    def sendOperationMode(self):
        pass
    def sendPreoperationMode(self):
        pass

class EduroMaxiReader:
    def __init__(self, fn):
        self.runconfig = list(getlogs(fn))
        self.run=0
        self.correction=Position(0,0,0)

    def startrun(self, i, startposition=None):
        if i<0:
            i=len(self.runconfig)+i
        print("selecting run %i of %i"%(i,len(self.runconfig)))
        self.run=i
        runnumber, position, run = self.runconfig[i]
        if startposition:
            self.startposition=startposition
            position=startposition
        else:
            self.startposition=position
        self.robot=EduroMaxi(can=DummyCan())
        self.robot.localisation = KalmanFilter(pose=position)
        self.waitforstart=True
        self.lastcable=False
        self.readers=[]
        self.names=[]
        for type, fn in run:
            if type=="CANlog":
                self.canreader=MsgGrouper(MsgDummyInterpreter(Msgparser(fn)), filter_origin=["can"])
            else:
                self.names.append(type)
                self.readers.append(textlogreader(fn))

    def nextrun(self):
        run=self.run+1
        if run >= len(self.runconfig):
            run=0
        self.startrun(run)

    def update(self):
        while True:
            time, can_msgs=self.canreader.next()
            for id,data in can_msgs:
                self.robot.updateEnc(id,data)
                #self.robot.updateEmergencyStop( id, data )
                #self.robot.updateCompass(id,data)
                self.robot.updateButtons(id,data)
                #self.robot.updateSharps( id,data )
                #self.robot.updateBattery( id, data )
            self.received=dict()
            for name, reader in zip(self.names, self.readers):
                self.received[name]=reader.next()
            if self.waitforstart:
                if self.lastcable and not self.robot.startCableIn:
                    self.waitforstart=False
                    break
                else:
                    self.lastcable=self.robot.startCableIn
            else:
                break
        return time, transponse(self.correction, self.robot.localisation.pose()), self.received

    def __iter__(self):
        return self

    def next(self):
        return self.update()

if __name__ == "__main__":
    robot=EduroMaxi(can=DummyCan())
    robot.localisation = KalmanFilter()
    for run, start, msgdict in MetaLogReader(sys.argv[1]):
        time, can_msgs=msgdict["CANlog"]
        for id,data in can_msgs:
            robot.updateEnc(id,data)
            robot.updateEmergencyStop( id, data )
            robot.updateCompass(id,data)
            robot.updateButtons(id,data)
            robot.updateSharps( id,data )
            robot.updateBattery( id, data )
        #print(robot.localisation.pose())
        if 'RFU620LOG' in msgdict and msgdict['RFU620LOG']:
            id, scans=msgdict['RFU620LOG']
            #print(robot.localisation.pose())
            for s in scans:
                #print(s.id, s.RSSI)
                pass
