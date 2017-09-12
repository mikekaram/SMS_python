# The code for changing pages was derived from: http://stackoverflow.com/questions/7546050/switch-between-two-frames-in-tkinter
# License: http://creativecommons.org/licenses/by-sa/3.0/	

import matplotlib
import time
import SMSLibrary as sms
import Tkinter as tk
import ttk
import sys

from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
import matplotlib.animation as animation
from matplotlib import style


matplotlib.use("TkAgg")
LARGE_FONT = ("Verdana", 12)
style.use("ggplot")



rate = 1 #rate in msec. Unfortunately we cannot give value less than 1 because FuncAnimation() takes interval as integer.
#If you find a way to do this, let us know!
curSamples = 0
posSamples = 0
currList = []
posList = []
timeListCurr = []
timeListPID = []
motorId = 5
currentLimit = 0
StartPID = False
StartCurr = False

def print_message():
    print("Usage: [sudo] python SMS_gui.py port_number \n")
    exit(0)

def animatePID(i):
    global posSamples, posList,timeListPID, motorId, Autoscale
    if StartPID and not StartCurr:
        pullData = float(sms.getPosition(motorId))
        # print(float(pullData))
        time = float(posSamples*rate)/1000
        if(len(posList) > 100):
            posList = posList[1:]
            timeListPID = timeListPID[1:]

        posList.append(int(pullData))
        timeListPID.append(time)
        a1.clear()
        if (not Autoscale.get()):
            a1.set_ylim([min(start,goal)-1000, max(start, goal) + 1000])
        a1.set_title("Position (ticks) - time (sec)")
        a1.set_xlabel("time (sec)")
        a1.set_ylabel("Position (ticks)")
        a1.plot(timeListPID, posList,marker='.',markersize=12)
        posSamples += 1

def animateCurrent(i):
    global curSamples
    global currList
    global timeListCurr
    global motorId
    if(StartCurr and not StartPID):
        pullData = sms.getCurrent(motorId)

        time = float(curSamples*rate)/1000
        if(len(currList) > 100):
            currList = currList[1:]
            timeListCurr = timeListCurr[1:]

        currList.append(int(pullData))
        timeListCurr.append(time)
        a2.clear()
        a2.set_ylim([-50, int(currentLimit)])
        a2.set_title("current (mA) - time (sec)")
        a2.set_xlabel("time (sec)")
        a2.set_ylabel("current (mA)")
        a2.plot(timeListCurr, currList)
        curSamples += 1


class SMSapp(tk.Tk):
    def __init__(self, *args, **kwargs):
        tk.Tk.__init__(self, *args, **kwargs)

        # tk.Tk.iconbitmap(self, default="clienticon.ico")
        tk.Tk.wm_title(self, "Super Modified Servo application")

        container = tk.Frame(self)
        container.grid(column = 1, row = 1)

        self.frames = {}

        for F in (StartPage,PIDGraph, CurrentGraph):
            frame = F(container, self)

            self.frames[F] = frame

            frame.grid(row=0, column=0, sticky="nsew")

        self.show_frame(StartPage)

    def show_frame(self, cont):
        frame = self.frames[cont]
        frame.tkraise()
        if(cont == PIDGraph or cont == CurrentGraph):
            frame.canvas.draw_idle()



class StartPage(tk.Frame):

    def __init__(self, parent, controller):
        tk.Frame.__init__(self, parent)
        label = tk.Label(self, text="Start Page", font=LARGE_FONT)
        label.grid(row=1,column=3,pady=10, padx=10)

        button2 = ttk.Button(self, text="Test PID",
                             command=lambda: controller.show_frame(PIDGraph))
        button2.grid(row=3,column=3)

        button3 = ttk.Button(self, text="Current Graph",
                             command=lambda: controller.show_frame(CurrentGraph))
        button3.grid(row=3,column=4)
        MLabel = ttk.Label(self, text="Motor ID: (now is "+str(motorId)+")")
        MLabel.grid(column=3,row=4,sticky=tk.W)
        MEntry = ttk.Entry(self)
        MEntry.grid(column=4,row=4,sticky=tk.W)
        MButton = ttk.Button(self, text="Change",
                             command=lambda: self.changeMotor(int(MEntry.get())))
        MButton.grid(column=5,row=4,sticky=tk.W)

    def changeMotor(self,motorID):
        global motorId
        motorId = motorID

# class PageOne(tk.Frame):
#     def __init__(self, parent, controller):
#         tk.Frame.__init__(self, parent)
#         label = tk.Label(self, text="Page One!!!", font=LARGE_FONT)
#         label.grid(pady=10, padx=10)
#
#         button1 = ttk.Button(self, text="Back to Home",
#                              command=lambda: controller.show_frame(StartPage))
#         button1.grid()
#
#         button2 = ttk.Button(self, text="Test PID",
#                              command=lambda: controller.show_frame(PIDGraph))
#         button2.grid()


class PIDGraph(tk.Frame):


    def __init__(self, parent, controller):
        global PEntry, IEntry, DEntry, SPointEntry, FPointEntry
        global motorId, Type, Direction,Autoscale
        Type = tk.StringVar()
        Type.set("Profiled")
        Direction = tk.StringVar()
        Direction.set("Absolute")
        Autoscale = tk.IntVar()
        Autoscale.set(1)
        tk.Frame.__init__(self, parent)
        label = tk.Label(self, text="Test PID", font=LARGE_FONT)
        label.grid(column=10,row=0)

        button1 = ttk.Button(self, text="Back to Home",
                             command=lambda: controller.show_frame(StartPage))
        button1.grid(column=0,row=1,sticky=tk.W)
        button2 = ttk.Button(self, text="Go to Current Graph",
                             command=lambda: controller.show_frame(CurrentGraph))
        button2.grid(column=0,row=2,sticky=tk.W)
        button3 = ttk.Button(self, text="Start Motor",
                             command= lambda: sms.start(motorId))
        button3.grid(column=0,row=3,sticky=tk.W)
        button4 = ttk.Button(self, text="Stop Motor",
                             command=lambda: sms.stop(motorId))
        button4.grid(column=0,row=4,sticky=tk.W)

        button3 = ttk.Button(self, text="Reset Errors",
                             command= lambda: sms.resetErrors(motorId))
        button3.grid(column=0,row=5,sticky=tk.W)
        button3 = ttk.Button(self, text="Start Move",
                             command= lambda: self.chooseMove())
        button3.grid(column=0,row=6,sticky=tk.W)

        button3 = ttk.Button(self, text="Halt",
                             command= lambda: self.doHalt())
        button3.grid(column=0,row=7,sticky=tk.W)
        button3 = ttk.Checkbutton(self, text="Auto Scale ?", variable=Autoscale,onvalue=1,offvalue=0)
        button3.grid(column=0,row=8,sticky=tk.W)
        R1 = ttk.Radiobutton(self, text="Direct", variable=Type, value="Direct")
        R1.grid(column=0,row=9,sticky=tk.W)
        R2 = ttk.Radiobutton(self, text="Profiled", variable=Type, value="Profiled")
        R2.grid(column=1,row=9,sticky=tk.W)
        R1 = ttk.Radiobutton(self, text="Absolute", variable=Direction, value="Absolute")
        R1.grid(column=0,row=10,sticky=tk.W)
        R2 = ttk.Radiobutton(self, text="Relative", variable=Direction, value="Relative")
        R2.grid(column=1,row=10,sticky=tk.W)
        PLabel = ttk.Label(self, text="P Value: (now is "+str(sms.getPIDgainP(motorId))+")")
        PLabel.grid(column=0,row=11,sticky=tk.W)
        PEntry = ttk.Entry(self)
        PEntry.grid(column=1,row=11,sticky=tk.W)
        PButton = ttk.Button(self, text="Change",
                             command=lambda: self.setP(controller,int(PEntry.get()),PIDGraph))
        PButton.grid(column=2,row=11,sticky=tk.W)
        ILabel = ttk.Label(self, text="I Value: (now is "+str(sms.getPIDgainI(motorId))+")")
        ILabel.grid(column=0,row=12,sticky=tk.W)
        IEntry = ttk.Entry(self)
        IEntry.grid(column=1,row=12,sticky=tk.W)
        IButton = ttk.Button(self, text="Change",
                             command=lambda: self.setI(controller,int(IEntry.get()),PIDGraph))
        IButton.grid(column=2,row=12,sticky=tk.W)
        DLabel = ttk.Label(self, text="D Value:(now is "+str(sms.getPIDgainD(motorId))+")")
        DLabel.grid(column=0,row=13,sticky=tk.W)
        DEntry = ttk.Entry(self)
        DEntry.grid(column=1,row=13,sticky=tk.W)
        DButton = ttk.Button(self, text="Change",
                             command=lambda: self.setD(controller,int(DEntry.get()),PIDGraph))
        DButton.grid(column=2,row=13,sticky=tk.W)
        SPointLabel = ttk.Label(self, text="Starting Point:")
        SPointLabel.grid(column=0,row=14,sticky=tk.W)
        SPointEntry = ttk.Entry(self)
        SPointEntry.grid(column=1,row=14,sticky=tk.W)
        FPointLabel = ttk.Label(self, text="Goal:")
        FPointLabel.grid(column=0,row=15,sticky=tk.W)
        FPointEntry = ttk.Entry(self)
        FPointEntry.grid(column=1,row=15,sticky=tk.W)
        canvasPID = FigureCanvasTkAgg(f1, self)
        canvasPID.show()
        canvasPID.get_tk_widget().grid(column=7,row=7,rowspan=7,columnspan=7)
        self.canvas = canvasPID

    def setP(controller,Pvalue, frame):
        if (Pvalue <= 0):
            print("Wrong P Value!")
        else:
            sms.setPIDgainP(motorId, Pvalue)
        controller.show_frame(frame)

    def setI(controller, IValue, frame):
        if (IValue <= 0):
            print("Wrong I Value!")
        else:
            sms.setPIDgainI(motorId, IValue)
        controller.show_frame(frame)

    def setD(controller, DValue, frame):
        if (DValue <= 0):
            print("Wrong D Value!")
        else:
            sms.setPIDgainD(motorId, DValue)
        controller.show_frame(frame)

    def chooseMove(self):
        global timeList, SPointEntry, FPointEntry, PEntry, IEntry, DEntry
        global StartPID, StartCurr, start, goal
        start = int(SPointEntry.get())
        goal = int(FPointEntry.get())
        if Type.get() == "Direct":
            if Direction.get() == "Absolute":
                sms.moveToAbsolutePosition(motorId, start)
                time.sleep(5)
                sms.moveToAbsolutePosition(motorId, goal)
            elif Direction.get() == "Relative":
                sms.moveToRelativePosition(motorId, start)
                time.sleep(5)
                sms.moveToRelativePosition(motorId, goal)
        elif Type.get() == "Profiled":
            if Direction.get() == "Absolute":
                sms.profiledMoveToAbsolutePosition(motorId, start)
                time.sleep(5)
                sms.profiledMoveToAbsolutePosition(motorId, goal)
            elif Direction.get() == "Relative":
                sms.profiledMoveToRelativePosition(motorId, start)
                time.sleep(5)
                sms.profiledMoveToRelativePosition(motorId, goal)
        StartPID = True
        StartCurr = False

    def doHalt(self):
        global StartPID
        StartPID = False



class CurrentGraph(tk.Frame):
    def __init__(self, parent, controller):
        tk.Frame.__init__(self, parent)
        label = tk.Label(self, text="Current Graph", font=LARGE_FONT)
        label.grid(row=0,column=10,pady=10, padx=10)

        button1 = ttk.Button(self, text="Back to Home",
                             command=lambda: controller.show_frame(StartPage))
        button1.grid(row=1,column=0,sticky=tk.W)
        button2 = ttk.Button(self, text="Go to PID Graph",
                             command=lambda: controller.show_frame(PIDGraph))
        button2.grid(row=2,column=0,sticky=tk.W)
        button3 = ttk.Button(self, text="Start Motor",
                             command= lambda: self.doStart())
        button3.grid(row=3,column=0,sticky=tk.W)
        button4 = ttk.Button(self, text="Stop Motor",
                             command=lambda: sms.stop(motorId))
        button4.grid(row=4,column=0,sticky=tk.W)

        button5 = ttk.Button(self, text="Reset Motor",
                             command=lambda: sms.reset(motorId))
        button5.grid(row=5,column=0,sticky=tk.W)

        canvasCurr = FigureCanvasTkAgg(f2, self)
        canvasCurr.show()
        canvasCurr.get_tk_widget().grid(column=7,row=7,rowspan=7,columnspan=7)
        self.canvas = canvasCurr

    def doStart(self):
        global StartCurr, StartPID
        sms.start(motorId)
        StartCurr = True
        StartPID = False

f1 = Figure(figsize=(7, 7), dpi=100)
a1 = f1.add_subplot(111)
f2 = Figure(figsize=(7, 7), dpi=100)
a2 = f2.add_subplot(111)
if(len(sys.argv) != 2):
    print_message()

sms.init(sys.argv[1])
while currentLimit == 0:
    currentLimit = sms.getCurrentLimit(motorId)
    time.sleep(0.02)
app = SMSapp()
# app.resizable(500,500)
aniPID = animation.FuncAnimation(f1, animatePID, interval=rate)
aniCurrent = animation.FuncAnimation(f2, animateCurrent, interval=rate)
app.mainloop()