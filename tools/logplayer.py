# vim: set fileencoding=utf-8 et sts=4 sw=4:
from __future__ import absolute_import, division, print_function, unicode_literals
from Tkinter import Toplevel, Label, Button, Scale, Frame
from logparser import EduroMaxiReader, getlogs
from viewer import tkimg
import os
from common import Position
from math import atan, sin, cos, pi

class SelcectDialog:
    def __init__(self, parent, label, options):
        top = self.top = Toplevel(parent)
        Label(top, text="label").grid(row=0,column=0)
        for i,o in enumerate(options):
            b = Button(top, text=o, command=lambda selection=o: self.select(selection))
            b.grid(row=i+1,column=0)
    def select(self, option):
        self.option=option
        self.top.destroy()

def ts(fn):
    return fn[4:-4].strip('_')

def listdir(logdir, verbose=False):
    for dirpath, dirnames, filenames in os.walk(logdir, topdown=True, onerror=None, followlinks=False):
        for fn in filenames:
            if fn.startswith('meta'):
                try:
                    runs=list(getlogs(os.path.join(dirpath,fn)))
                    if len(runs):
                        yield ts(fn), dirpath, fn
                except:
                    if verbose:
                        print("wrong metafile", os.path.join(dirpath,fn))


class Logplayer(Frame):
    def __init__(self, root, logfn, viewer, analyzator=None, logdir=None, **kwargs):
        self.root=root
        self.logdir = logdir
        self.analyzator = analyzator
        if not self.logdir:
            self.logdir=os.path.join(os.path.dirname(os.path.dirname(__file__)),'log')
        Frame.__init__(self, **kwargs)
        self.view=viewer(self, x=800, y=600, onLeft=self.clickL, onRight=self.clickR)
        self.createWidgets()
        self.bind_all("<Left>", self.prev)
        self.bind_all("<Right>", self.next)

        self.view.redraw()
        self.root.update()
        if not logfn:
            logfn = self.logSelectDialog()
        self.dirname, self.filename = os.path.split(logfn)
        self.startRun(logfn,0)
        self.view.redraw()
        self.root.after(10,self.view.zoomExtens)
        self.running=False

    def logSelectDialog(self):
            self.root.update()
            timestamps, dirs, fns = zip(*sorted(listdir(self.logdir), reverse=True))
            o=SelcectDialog(self.root, "select log", timestamps)
            self.root.wait_window(o.top)
            index=timestamps.index(o.option)
            return os.path.join(dirs[index], fns[index])

    def createWidgets(self):
        self.controlFrame=Frame(self)
        self.posScale = Scale(self.controlFrame, orient='horizontal',
                           length=210,
                           from_=0, tickinterval=0,
                           command=self.pos_callback,
                           showvalue=0,sliderlength=10, resolution=1)
        #self.posScale.bind('<Button-1>',self.pos_start_drag)
        #self.posScale.bind('<ButtonRelease-1>', self.pos_set)

        self.playB=Button(self.controlFrame, text="play", command=self.playToggle)
        self.nextB=Button(self.controlFrame, text="next", command=self.nextRun)
        self.prevB=Button(self.controlFrame, text="prev", command=self.prevRun)
        self.camera=tkimg(self, width=640, height=512)
        self.controlFrame.grid(row=0,column=1, sticky="nw")
        self.view.grid(row=1, column=1, sticky="nsew")
        self.grid_columnconfigure(1,weight=1)
        self.grid_rowconfigure(1,weight=1)
        self.camera.grid(row = 1, column = 2, sticky = 'ne')
        self.playB.grid(row=0, column=1)
        self.prevB.grid(row=0, column=2)
        self.nextB.grid(row=0, column=3)
        self.posScale.grid(row=0, column=0)


    def playToggle(self, *args):
        if self.running:
            self.root.after_cancel(self.cycleId)
        else:
            self.cycleId=self.root.after(20, self.showrun)
        self.running=not self.running

    def nextRun(self, *args):
        self.changeRun(1)

    def prevRun(self, *args):
        self.changeRun(-1)

    def changeRun(self, step):
        if self.running:
            self.root.after_cancel(self.cycleId)
            self.running=False
        run=self.loggenerator.run+step
        if run >= len(self.loggenerator.runconfig) or run<0:
            timestamps, dirs, fns = zip(*sorted(listdir(self.logdir)))
            index=fns.index(self.filename)+step
            if 0<=index<len(fns):
                run = 0 if step==1 else -1
                self.startRun(os.path.join(dirs[index], fns[index]), run)
        else:
            self.startRun(os.path.join(self.dirname, self.filename), run)




    def restart(self, keeppostion=True, startposition=None):
        self.loggenerator.startrun(self.loggenerator.run, startposition=startposition)
        self.data=list(self.loggenerator)
        if not keeppostion:
            self.position=0
        self.posScale.set(self.position)
        self.showData(self.position)

    def startRun(self, fn, run, keeppostion=False):
        self.dirname, self.filename = os.path.split(fn)
        print("starting log %s run %i"%(fn,run))
        self.loggenerator=EduroMaxiReader(fn)
        self.loggenerator.startrun(run)
        self.root.title("%s %s run %i"%(self.dirname,self.filename, self.loggenerator.run+1))
        self.data=list(self.loggenerator)
        if not keeppostion:
            self.position=0
        self.posScale.set(self.position)
        self.showData(self.position)
        self.posScale.configure(to_= len(self.data)-1)
        self.view.reset()
        self.camera.clear()

    def showData(self,position):
        self.position=position
        self.posScale.set(position)
        time, pose, msgs = self.data[position]
        self.view.MsgListener(time, pose, msgs)
        #if self.analyzator:
        #    self.analyzator(time, pose, msgs)
        self.view.robot.update(*pose)
        self.view.redraw_lasers()
        self.view.robot.redraw(self.view)
        if msgs['CAMERALOG']:
            _, shot = msgs['CAMERALOG']
            if shot:
                shot = os.path.join(self.dirname, shot.split('/')[-1])
                self.camera.showImg(shot)

    def clickL(self, x, y):
        #self.loggenerator.correction=Position(self.loggenerator.correction.x+x-self.view.robot.x, self.loggenerator.correction.y+y-self.view.robot.y, self.loggenerator.correction.a)
        #print(self.loggenerator.correction)
        A=self.data[self.position][1]
        S=self.data[0][1]
        #self.restart(keeppostion=True, startposition=Position(S.x+x-A.x, S.y+y-A.y, S.a))
        print("left", x,y)

    def clickR(self, x, y):
        #try to modify start position so that actual position heading is in direction of this click

        #actual position
        A=self.data[self.position][1]
        #start position
        S=self.data[0][1]
        #polar vector from actual to start
        dist=((A.x-S.x)**2+(A.y-S.y)**2)**0.5
        if S.x==A.x:#when x distance is 0 then angle +-90 (when y dist is 0 then it doesnt matter
            beta=-pi/2 if S.y>=A.y else pi/2
        else:
            beta=atan((A.y-S.y)/(A.x-S.x))
        #angle difference from actual position to click
        #TODO solve +-90 same as above
        delta=A.a-atan((A.y-y)/(A.x-x))

        newStart = Position(A.x+cos(beta+delta)*dist, A.y+sin(beta+delta)*dist, S.a-delta)
        #self.restart(keeppostion=True, startposition=newStart)
        print("right", x, y)

    def showrun(self):
        pos=self.position+1
        if pos<len(self.data):
            self.showData(pos)
            self.cycleId=self.root.after(20, self.showrun)
        else:
            self.running=False

    def next(self, event):
        pos=self.position+1
        if pos<len(self.data):
            self.posScale.set(pos)

    def prev(self, event):
        pos=self.position-1
        if pos>=0:
            self.posScale.set(pos)

    def pos_callback(self, pos, *args):
        if not self.running:
            self.showData(int(pos))
