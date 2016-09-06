#!/usr/bin/python
# vim: set fileencoding=utf-8 et sts=4 sw=4:
from __future__ import absolute_import, division, print_function, unicode_literals
"""
  Laser SICK LMS100-10000/TIM310
    autodetection: 192.168.1.1, ports 2111 (aux) and 2112 (host)
    192.168.3.1:2111 s maskou 255.255.0.0
  usage:
      laser.py <IP|USB> [--config|<number of scans>]
"""

from socket import socket, AF_INET, SOCK_STREAM
from time import sleep
from collections import namedtuple
from itertools import groupby

STX = chr(2)
ETX = chr(3)


scan_complete_nt=namedtuple('scan_complete',('idlen', 'id', 'antena', 'RSSI1', 'RSSI2', 'RSSI3', 'RSSI4', 'power1', 'power2', 'power3', 'power4'))
scan_nt=namedtuple('scan',('id', 'RSSI', 'power'))

class RFU620():
    def __init__( self, host, port, verbose=False):
        self.socket = socket(AF_INET, SOCK_STREAM)
        self.socket.connect((host, port))
        self._reader = self.msg_generator()
        self.verbose=verbose

    def __del__( self ):
        self.socket.close()

    def msg_generator(self):
        data=r''
        while True:
            data += self.socket.recv(1024)
            if data:
                blocks=data.split(ETX)
                #if data ends with ETX, last block will be ''
                # otherwise data ends with unterminated packet, keep it in buffer
                data=blocks.pop()
                for msg in blocks:
                    if not msg:
                        continue
                    try:
                        garbage, msg = msg.rsplit(STX,1)
                        if self.verbose and garbage:
                            print("Some unparsed msg: "+repr(garbage))
                        yield msg
                    except ValueError:
                        if self.verbose:
                            print("No STX found in msg: "+repr(msg))
                        pass

    def receive(self):
        return self._reader.next()

    def sendCmd(self, cmd):
        self.socket.send(STX + cmd + ETX)
        return self.receive()

    def readPowerConfig(self):
        print(self.sendCmd("sRN ADconfig0"))
        #sRA ADconfig0 1 64 96 96 2000 1 96 A 0


    def _writePowerConfig(self, enabled, dwelltime, rpower, wpower, inv_rounds, priority, APCmin, APCstep):
        # to adjust power, we need to set access mode to 3 or higher
        #secret password copied form laser docs
        a=self.sendCmd("sMN SetAccessMode 3 F4724744")#7A99FDC6")
        assert a=="sAN SetAccessMode 1", a
        a=self.sendCmd("sWN ADconfig0 "+' '.join(["%X"%val for val in (enabled, dwelltime, rpower, wpower, inv_rounds, priority, APCmin, APCstep, 0)]))
        assert a=="sWA ADconfig0", a
        a=self.sendCmd("sMN Run")
        assert a=="sAN Run 1", a



    def setpower(self, power):
        #accepts power between 30-240
        assert 30<=power<=240, "Power must be between 30 and 240 (3-24 dBm)"
        self._writePowerConfig( enabled=1, dwelltime=100, rpower=power, wpower=power, inv_rounds=0x2000, priority=1, APCmin=power, APCstep=10)

    def scan( self ):
        reply=self.sendCmd("sMN IVSingleInv F")
        result=[]
        if reply.startswith("sAN IVSingleInv"):
            reply=reply.split(' ')
            success=int(reply[2])
            count=int(reply[3])
            reply=reply[4:]
            while reply:
                # idlen, id, antena, RSSI1, RSSI2, RSSI3, RSSI4, power1, power2, power3, power4
                complete=scan_complete_nt(*map(lambda x: int(x,16), reply[:11]))
                # we are reading from antena 1, RSSI1 and power1
                assert complete.antena==1
                result.append(scan_nt(id=complete.id, RSSI=-(0xffff-complete.RSSI1)/10., power=complete.power1/10.))
                #continue to next sample
                reply=reply[11:]
        elif self.verbose:
            print("unexpected answer for scan: "+repr(reply))
        return result

def distance(RSSI, power):
    #RSSI (dBm) = -10n log10(d) + A
    c=2.5 #(n ranges from 2 to 4)
    d = 10. ** ((-RSSI - power) / (10.*c))
    return d

from math import log
if __name__ == "__main__":
    rf=RFU620('192.168.3.1', 2111, verbose=True)
    #rf.readPowerConfig()
    """
    for power in (30,100, 230,231, 240, 241):
        try:
            rf.setpower(power)
            print("Set %i power"%power)
        except AssertionError:
            print("%i power is not allowed"%power)
    """

    rf.setpower(150)
    di=33.
    while True:
        for power in (240,):#(50,100, 150, 200, 240):
            rf.setpower(power)
            print('power=%.1f, '%(power/10.), end='')
            samples=[]
            for i in range(10):
                samples+=rf.scan()
                print('.', end='')
            if not samples:
                print('')
                continue
            samples.sort(key=lambda x:x.id)
            for id, groups in groupby(samples, key=lambda x:x.id):
                ids, rssis, powers=zip(*groups)
                rssi=sum(rssis)/len(rssis)
                power=powers[0]
                n=(-rssi-power)/log(di,10)/10.
                print("r=%4.1f, d=%.2fm, n=%.2f; "%(rssi, distance(rssi, power), n), end='')
            print('')
            sleep(0.2)
