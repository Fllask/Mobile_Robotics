#!/Library/Frameworks/Python.framework/Versions/3.8/bin/python3.8
#the line above makes it more nicely executable on my mac, PATH may differ on different machines

# general includes (os stuff so that we can do multiprocesses)
import os #to access os parameters (Process Ids mostly)
from multiprocessing import * # I use the multiprocessing library for parallelism ==> https://docs.python.org/3.8/library/multiprocessing.html
import time #to time routines and have some idea of what is slow
import tkinter as tk #the windows are handled by tkinter => https://docs.python.org/3.8/library/tkinter.html

#bindings from matplotlib to tk
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure

#these are our modules
from Utilities import Utilities
from Global import Global

"""

    The way this file works is we define a bunch of functions that in turn 
    get instanciated as different processes by the multiprocess module, this allows
    us to implement true parralelism (something that threads are not able to do in cPython)

"""

# Image processing process
def imageProc():
    #process some images idk not my part
    time.sleep(0.1)

# Handles the windows and the overlays:
def display():
    #spawn windows 
    time.sleep(0.1)

#this class handles the math
class Compute():
    def run(self,stopSignal):
        mainLoopProcess = Process(target=self.mainLoop, args=(stopSignal,))
        mainLoopProcess.start()

    def mainLoop(self,stopSignal):
        ut = Utilities()
        glob = Global( ut.TestMap(),(20,20),(80,80) )  
        while stopSignal.value == False:
            time.sleep(0.1)
            glob.computePaths()



#this class handles the interface (it is the required tkInter class)
class Window(tk.Frame):
    def __init__(self, stopSignal, master=None):
        #initializin g the window
        super().__init__(master)
        self.master = master
        self.pack()

        self.w = 1000
        self.h = 1000

        self.create_widgets()
        self.stopSignal = stopSignal

    def create_widgets(self):
        # self.canvas = tk.Canvas(self.master, width=self.w, height=self.h)
        # self.canvas.pack()
        
        fig = Figure(figsize=(100,100),dpi = 10)
        self.canvas = FigureCanvasTkAgg(fig, self)

        self.canvas.get_tk_widget().pack(side=tk.BOTTOM, fill=tk.BOTH, expand=True)

        self.canvas._tkcanvas.pack(side=tk.TOP, fill=tk.BOTH, expand=True)

    def quit_window(self):
        print("QUITTING")
        self.stopSignal.value = True
        self.master.destroy()

# main function, root of all the program
if __name__ == '__main__':
    
    
    
    stopSignal = Value('b',False)

    #initializing the window object
    root = tk.Tk()
    app = Window(stopSignal, master = root)
    
    root.protocol("WM_DELETE_WINDOW",app.quit_window)

    #initializing the compute object
    compute = Compute()
    
    #running both main loops
    compute.run(stopSignal)
    app.mainloop()
        

