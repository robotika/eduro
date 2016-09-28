# vim: set fileencoding=utf-8 et sts=4 sw=4:
from __future__ import absolute_import, division, print_function, unicode_literals
from Tkinter import Tk
import sys
from logplayer import Logplayer
from viewer import Viewer, RobotView
from rfu_calibration import getDistanceToTag

class Sick2016(Viewer):
    def __init__(self, root, robotx=0, roboty=0, robota=0, **kwargs):
        self.robot=RobotView(front=0.15, rear=0.15, left=0.15, right=0.15, wheeldistance=0.315, wheeldiam=0.25, wheelwidth=0.05, x=robotx, y=roboty, a=robota)
        Viewer.__init__(self, root, **kwargs)

    def redraw(self):
        self.canvas.delete("all")
        self.draw_playfield()
        self.robot.draw(self)
        self.canvas.configure(scrollregion = self.canvas.bbox("all"))


    def draw_playfield(self):
        self.create_line([0, 0, 13.,0, 13., 7., 0, 7., 0, 0], activefill="red",fill='black',width=1),
        self.create_polygon([(0., 2.), (3., 2.), (3., 5.), (0., 5.)], fill="cyan")
        self.create_polygon([(4., 1.), (9., 1.), (9., 6.), (4., 6.)], fill="blue")
        self.create_polygon([(10., 2.), (13., 2.), (13., 5.), (10., 5.)], fill="green")
        self.create_polygon([(3., 1.), (5., 1.), (5., 3.), (3., 3.)], fill="yellow")

        crosssize=0.025
        for x in range(14):
            for y in range(8):
                self.create_line([x-crosssize, y, x+crosssize, y], activefill="red",fill='black',width=2)
                self.create_line([x, y-crosssize, x, y+crosssize], activefill="red",fill='black',width=2)

class Analyzator:
    def __init__(self, outfn):
        self.output=open(outfn, 'w')
        self.output.write(';'.join(("time", "x", "y", "a", "tagid", "RSSI", "computed"))+'\n')

    def update(self, time, pose, msgs):
        if 'RFU620LOG' in msgs and msgs['RFU620LOG']:
            _, scans  = msgs['RFU620LOG']
            for scan in scans:
                distance = getDistanceToTag(pose, scan.id)
                data=[time, pose[0], pose[1], pose[2], scan.id, scan.RSSI, distance ]
                self.output.write(';'.join(data)+'\n')

if __name__=="__main__":
    fn=None
    if len(sys.argv)>1:
        fn=sys.argv[1]
    root=Tk()
    analyzator=Analyzator(outfn="log.csv")
    a=Logplayer(root, fn, viewer = Sick2016, analyzator=analyzator.update)
    a.grid()
    root.mainloop()




