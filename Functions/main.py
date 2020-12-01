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

from Functions.Thymio import Thymio

#these are our modules
from Functions.Utilities import Utilities
from Functions.Global import Global
import Functions.Vision as v
from Functions.Robot import Robot
from Functions.Filtering import Filtering
# import Robot as r

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

   
    """ Displays processed data """
    def display(self):
        #projecting the image
        tr_img = cv2.warpPerspective(self.img, self.vis.trans, (500,500))

        font = cv2.FONT_HERSHEY_SIMPLEX 

        #plotting the obstacles detected
        cv2.drawContours(tr_img, self.vis.getMap(downscale=False), -1, (0,255,0), 2)
        
        #plotting the gobal navigation path
        path = self.g.path
        for i in range(1,len(path)) :
            cv2.line(tr_img,(int(path[i][0]*5),int(path[i][1]*5)),(int(path[i-1][0]*5),int(path[i-1][1]*5)),(0,0,0),thickness=2)
        
        ## plotting the robot's position
        if ~(self.rob == False):
            cv2.circle(tr_img,(int(self.rob[0]*5),int(self.rob[1]*5)),60,(0,0,255),thickness=4)
            tr_img = cv2.putText(tr_img, 'Robot coordinates : ' + str(self.rob), (int(self.rob[0]*10),int(self.rob[1]*10)), font,  1, (0,0,255), 1, cv2.LINE_AA) 
        ## plotting the goal
        if ~(self.rob == False):
            cv2.circle(tr_img,(int(self.stop[0]*5),int(self.stop[1]*5)),60,(255,0,0),thickness=4)
            tr_img = cv2.putText(tr_img, 'Goal coordinates : ' + str(self.stop), (int(self.stop[0]*5),int(self.stop[1]*5)), font,  1, (0,0,255), 1, cv2.LINE_AA) 

        return tr_img
        
    
        
        
    """ Main loop for vision """
    def mainLoop(self,d):
        if self.verbose:
            print("Starting vision + global navigation main loop")
       
        #getting the camera input
        cap = cv2.VideoCapture(2)
        #get the first frame to test
        ret, self.img = cap.read()
        if ~ret:
            if self.verbose:
                print("frame droped")
                
         # initialization of the vision object
        t0 = time.process_time()
        initfailed = True
        while(initfailed):
            ret,frame = cap.read()
            self.vis = v.Vision(frame, "ANDROID FLASK")
            initfailed = self.vis.invalid
            cv2.imshow("frame",frame)
            cv2.waitKey(1)
        
        cv2.destroyWindow("frame")
        if self.verbose:
            print("Initial Mapping Time : " + str(time.process_time()-t0))
            
            
        #querying the aim coordinates
        coordAim, validcoord = self.vis.returnDynamicAim()
        if validcoord:
            self.stop = tuple(coordAim)
        else:
            #default aim coodinates
            self.stop = (95.,5.)
            
            
        

        # querying robot coordinates
        t0 = time.process_time()
        rbt_pos,ret = self.vis.returnDynamicCoordinates() ## getting robot coordinate
        
        if rbt_pos == False:
            self.rob = False
        else:
            self.rob = tuple(rbt_pos[0:2])
            
        d['pos'] = rbt_pos
        
        if self.verbose:
            print("Initial Robot_Pos Estimation Time : "+str(time.process_time()-t0))
        
        #initializing the path planning module
        t0 = time.process_time()
        self.obstacles = self.vis.getMap()
        d['map'] = self.obstacles
        
        
        self.g = Global(self.obstacles,False,self.stop)
        
        if ~(self.rob == False):
            self.g.start = self.rob
            self.path = self.g.returnPath()
            d['path'] = self.g.path
        else:
            if self.verbose:
                print("Robot not found")
                
        if self.verbose:
            print("Initial Path Planning Time : "+str(time.process_time()-t0))

        while True: 
            t0 = time.process_time() #we time each loop to get an idea of performance

            # loading new image
            ret, self.img = cap.read()
            self.vis.setframe(self.img) 
            
            ## getting robot coordinates
            rbt_pos, self.pos_valid = self.vis.returnDynamicCoordinates() 
            if ~(rbt_pos == False):
                self.rob = tuple(rbt_pos[0:2])
            else:
                self.rob = False
                
            d['pos'] = rbt_pos
            if self.verbose:
                print(d['pos'])
            
            ## computing path
            # self.g.start = self.rob
            # self.path = self.g.plotPath(plotGraph=False,plotMap=False)

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

        # Initialise robot class

        thym = Utilities.init_robot()

        # Initialise Filtering class

        filter = Utilities.init_filter()

        init = False
        go=1

        while go:

            tps1 = time.monotonic()

            ################################################################################
            # Initialise robot with a verified global path and a verified initial position #
            ################################################################################

            while not init : 
                Robot.initialisation(sef,d['path'],d['pos'])

            #########################################################################
            # get the position of the robot given by the camera when it is possible #
            #          if not possible set the updateWithCam bolean to False        # 
            #########################################################################

            pos_cam = d['pos'] if d['pos'] else np.array[[0],[0]]

            # Enter the robot FSM once initialised correctly

            thym.thymio(Ts,th)


            #[give : x,y,theta,vr,vl] to the filter : 
            
            robot_states = thym.get_states()

            time.sleep(0.1)
    
            # get the measurements from the camera : 

            # get our pos with the filter
            if thym.astolfi==1:
                filter.compute_kalman(pos_cam,robot_states,self.th,Ts,False)

            

            tps2 = time.monotonic()
            Ts=tps2-tps1

            if thym.p<3 and thym.node==len(thym.global_path)-2:
                self.th.set_var("motor.left.target", 0)
                self.th.set_var("motor.right.target", 0)
                print('FININSH!!!!')
                tfinal=time.monotonic()
                go = 0


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
    verbose = True
    for i in range (1,len(sys.argv)):
        if sys.argv[i] == "v":
            print("Running Verbose Mode")
            verbose = True

    try:
        ctrl = RobotControl(verbose,"COM3")
    except:
        sys.exit(1)
    
    
    manager = Manager()

    d = manager.dict()
    d['pos'] = False
    d['path'] = False
    d['map'] = False
    d['goal'] = False
    d['vtime'] = "0"

    cmptVis = ComputeVision(False)

    cmptVis.run(d)
    ctrl.run(d)
    cmptVis.join()
    ctrl.join()
