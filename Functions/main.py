#!/Library/Frameworks/Python.framework/Versions/3.8/bin/python3.8
#the line above makes it more nicely executable on my mac, PATH may differ on different machines

# general includes (os stuff so that we can do multiprocesses)
import os #to access os parameters (Process Ids mostly)
import sys
from multiprocessing import * # I use the multiprocessing library for parallelism ==> https://docs.python.org/3.8/library/multiprocessing.html
import time #to time routines and have some idea of what is slow
import cv2
print('OpenCL available:', cv2.ocl.haveOpenCL())
if cv2.ocl.haveOpenCL():
    cv2.ocl.setUseOpenCL(True)
import numpy as np
import math
import copy

#these are our modules
from Utilities import Utilities
from Global import Global
import Vision as v

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
class ComputeVision():
    """ Constructor of the ComputeVision class"""
    def __init__(self,verbose):
        self.verbose = verbose


    """ Starts the vision process """
    def run(self):
        mainLoopProcess = Process(target=self.mainLoop, args=())
        mainLoopProcess.start()

    """ Image Loading Function """
    def loadImg(self):
        t0 = time.process_time()
        input_path = '../sample_pictures/test_set_2/06.jpg'
        img = v.get_image(input_path)
        if self.verbose: 
            print("Image Query Time : "+str(time.process_time()-t0))
        return img

    """ Displays processed data """
    def display(self):
        tr_img = cv2.warpPerspective(self.img, self.vis.trans, (1000,1000))

        path = self.g.path
        for i in range(1,len(path)) :
            cv2.line(tr_img,(int(path[i][0]*10),int(path[i][1]*10)),(int(path[i-1][0]*10),int(path[i-1][1]*10)),(255,0,0),thickness=3)
        cv2.drawContours(tr_img, self.vis.getMap(downscale=False), -1, (0,255,0), 3)

        return tr_img

        
    """ Main loop for vision """
    def mainLoop(self):
        # initialization of the vision object
        self.img = self.loadImg()
        t0 = time.process_time()
        self.vis = v.Vision(self.img)
        if self.verbose:
            print("Initial Mapping Time : " + str(time.process_time()-t0))

        # querying robot coordinates
        t0 = time.process_time()
        rbt = self.vis.returnDynamicCoordinates() ## getting robot coordinate
        self.rob = (rbt[0][0]/10,rbt[0][1]/10)
        if self.verbose:
            print("Initial Robot_Pos Estimation Time : "+str(time.process_time()-t0))
        
        #initializing the path planning module
        t0 = time.process_time()
        self.obstacles = self.vis.getMap()
        self.g = Global(self.obstacles,(float(self.rob[0]),float(self.rob[1])),(5.,5.))
        self.path = self.g.plotPath(plotGraph=False,plotMap=False)
        if self.verbose:
            print("Initial Path Planning Time : "+str(time.process_time()-t0))

        while True: 
            t0 = time.process_time() #we time each loop to get an idea of performance

            # loading new image
            self.img = self.loadImg() 
            self.vis.setframe(self.img) 
            
            ## getting robot coordinates
            rbt = self.vis.returnDynamicCoordinates() 
            self.rob = (rbt[0][0]/10,rbt[0][1]/10)
            
            ## computing path
            self.path = self.g.plotPath(plotGraph=False,plotMap=False)

            ## displaying whatever was computed
            disp = self.display()
            cv2.imshow('frame',disp)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break      

            
            if self.verbose:
                print("Full Vision Loop : "+str(time.process_time()-t0))
            

# main function, root of all the program
if __name__ == '__main__':


    """ Parsing stdin """
    verbose = False
    for i in range (1,len(sys.argv)):
        if sys.argv[i] == "v":
            print("Running Verbose Mode")
            verbose = True

    
    cmptVis = ComputeVision(verbose)

    cmptVis.run()
    

    # main loop
    # while True:
        # t0 = t0 = time.process_time()
        # img = loadImg()
        # vis.setframe(img)
        # rbt = vis.returnDynamicCoordinates() ## getting robot coordinate
        # rob = rbt[0] 
        # path = g.plotPath(plotGraph=False,plotMap=False)
        # if verbose:
        #     print("Full Vision Loop : "+str(time.process_time()-t0))


    """ main display function needs to be wrapped in utilities this is too ugly"""
    # cv2.drawContours(img_real, obstacles, -1, (0,255,0), 3)
    # for p in obstacles:
    #     for corner in p:
    #         cv2.circle(img_real, tuple(corner.reshape(2)), 5, (255,255,0), thickness=1, lineType=8, shift=0)
    # pt1 = (int(rob[0]), int(rob[1]))
    # pt2 = (int(rob[0]+math.cos(rob[2])*100), int(rob[1]+math.sin(rob[2])*100))
    # cv2.line(img_real,pt1,pt2,(128,128,0),thickness=3)
    # for i in range(1,len(path)) :
    #     cv2.line(img_real,(int(path[i][0]*10),int(path[i][1]*10)),(int(path[i-1][0]*10),int(path[i-1][1]*10)),(255,0,0),thickness=3)
    # cv2.circle(img_real,pt1,10,(128,128,0),thickness = 4)
    # cv2.namedWindow('map',cv2.WINDOW_NORMAL)
    # cv2.resizeWindow('map', 600,600)
    # cv2.imshow('map', img_real)
    # cv2.waitKey(0)
    # cv2.destroyAllWindows()

    """ BREAK """


    # stopSignal = Value('b',False)

    # #initializing the window object
    # root = tk.Tk()
    # app = Window(stopSignal, master = root)
    
    # root.protocol("WM_DELETE_WINDOW",app.quit_window)

    # #initializing the compute object
    # compute = Compute()
    
    # #running both main loops
    # compute.run(stopSignal)
    # app.mainloop()
        

