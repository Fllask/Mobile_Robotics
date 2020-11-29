#!/Library/Frameworks/Python.framework/Versions/3.8/bin/python3.8
#the line above makes it more nicely executable on my mac, PATH may differ on different machines

"""

    The way the main module works is we define a bunch of functions in classes that in turn 
    get instanciated as different processes by the multiprocess module, this allows
    us to implement true parralelism (something that threads are not able to do in cPython)

"""

# general includes (os stuff so that we can do multiprocesses)
import os #to access os parameters (Process Ids mostly)
import sys
from multiprocessing import * # I use the multiprocessing library for parallelism ==> https://docs.python.org/3.8/library/multiprocessing.html
import time #to time routines and have some idea of what is slow
import cv2
if cv2.ocl.haveOpenCL():
    cv2.ocl.setUseOpenCL(True)
import numpy as np
import math
import copy

from Thymio import Thymio

#these are our modules
from Utilities import Utilities
from Global import Global
import Vision as v

""" 
    ComputeVision:
    Handles the vision and global planning as a process
    @autor: Titou
"""
class ComputeVision():
    """ Constructor of the ComputeVision class"""
    def __init__(self,verbose):
        self.verbose = verbose


    """ Starts the vision process """
    def run(self,d):
        self.mainLoopProcess = Process(target=self.mainLoop, args=(d,))
        self.mainLoopProcess.start()
    
    def join(self):
        self.mainLoopProcess.join()

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

        cv2.drawContours(tr_img, self.vis.getMap(downscale=False), -1, (0,255,0), 3)

        path = self.g.path
        for i in range(1,len(path)) :
            cv2.line(tr_img,(int(path[i][0]*10),int(path[i][1]*10)),(int(path[i-1][0]*10),int(path[i-1][1]*10)),(255,0,0),thickness=3)

        return tr_img

        
    """ Main loop for vision """
    def mainLoop(self,d):
        if self.verbose:
            print("Starting vision + global navigation main loop")
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
        d['pos'] = self.rob
        if self.verbose:
            print("Initial Robot_Pos Estimation Time : "+str(time.process_time()-t0))
        
        #initializing the path planning module
        t0 = time.process_time()
        self.obstacles = self.vis.getMap()
        d['map'] = self.obstacles
        self.g = Global(self.obstacles,(float(self.rob[0]),float(self.rob[1])),(5.,5.))
        self.path = self.g.plotPath(plotGraph=False,plotMap=False)
        d['path'] = self.path
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
            d['pos'] = self.rob
            
            ## computing path
            #self.path = self.g.plotPath(plotGraph=False,plotMap=False)

            ## displaying whatever was computed
            disp = self.display()
            cv2.imshow('frame',disp)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break      

            
            if self.verbose:
                print("Full Vision Loop : "+str(time.process_time()-t0))
            d['vtime']=str(time.process_time()-t0)


""" 
    RobotControl:
    Handles the control of the robot as a process (fast loop)
    @autor: Titou
"""
class RobotControl():
    """ Constructor of the RobotControl class """
    def __init__(self,verbose,address):
        self.verbose = True
        self.address = address

        

    """ Starts the control process """
    def run(self,d):
        self.mainLoopProcess = Process(target=self.mainLoop, args=(d,))
        self.mainLoopProcess.start()

    def join(self):
        self.mainLoopProcess.join()

    """ Main control loop """
    def mainLoop(self,d):

        t0 = time.process_time()
        if self.verbose:
            print("Starting control main loop")
        t = 0        

        if self.verbose:
            print("Connecting to Thymio at address "+self.address+" ... ")
        try:
            self.th = Thymio.serial(port=self.address, refreshing_rate=0.1)
            time.sleep(3)
            self.th.set_var_array("leds.top", [0, 0, 0])
            if self.verbose:
                print("connection successful !")
        except:
            print("Unexpected error:", sys.exc_info()[0])
            return

        if self.verbose:
            print("Starting main loop")
        while True:
            rt = time.process_time() - t0
            t0 = time.process_time()
            if int(round(time.process_time(),0)) > t:
                t = int(round(time.process_time(),0))
                nd = dict(d)
                if self.verbose:
                    print("control period : " + str(rt))
                    print("vision period : "+d["vtime"])
                    print("position of robot : "+str(nd['pos']))

            

"""
    main function, root of all the program
    @autor: Titou
"""
if __name__ == '__main__':

    print('OpenCL available:', cv2.ocl.haveOpenCL())


    """ Parsing stdin """
    verbose = False
    for i in range (1,len(sys.argv)):
        if sys.argv[i] == "v":
            print("Running Verbose Mode")
            verbose = True

    try:
        ctrl = RobotControl(verbose,"/dev/cu.usbmodem141401")
    except:
        sys.exit(1)

    manager = Manager()

    d = manager.dict()
    d['pos'] = False
    d['path'] = False
    d['map'] = False
    d['vtime'] = "0"

    cmptVis = ComputeVision(False)

    cmptVis.run(d)
    ctrl.run(d)
    cmptVis.join()
    ctrl.join()