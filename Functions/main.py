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
from Robot import Robot
from Filtering import Filtering
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
        if path != False:
            for i in range(1,len(path)) :
                cv2.line(tr_img,(int(path[i][0]*5),int(path[i][1]*5)),(int(path[i-1][0]*5),int(path[i-1][1]*5)),(0,0,0),thickness=2)
        
        ## plotting the robot's position
        if not (self.rob == False):
            cv2.circle(tr_img,(int(self.rob[0]*5),int(self.rob[1]*5)),60,(0,0,255),thickness=4)
            tr_img = cv2.putText(tr_img, 'Robot coordinates : ' + str(self.rob), (int(self.rob[0]*10),int(self.rob[1]*10)), font,  1, (0,0,255), 1, cv2.LINE_AA) 
        ## plotting the goal
        if not (self.rob == False):
            cv2.circle(tr_img,(int(self.stop[0]*5),int(self.stop[1]*5)),60,(255,0,0),thickness=4)
            tr_img = cv2.putText(tr_img, 'Goal coordinates : ' + str(self.stop), (int(self.stop[0]*5),int(self.stop[1]*5)), font,  1, (0,0,255), 1, cv2.LINE_AA) 

        return tr_img
        
    
        
        
    """ Main loop for vision """
    def mainLoop(self,d):
        if self.verbose:
            print("Starting vision + global navigation main loop")
       
        #getting the camera input
        cap = cv2.VideoCapture(0)
        #get the first frame to test
        ret, self.img = cap.read()
        if not ret:
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
        
        if not (self.rob == False):
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
            if not (rbt_pos == False):
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

        while d['path'] == False or d['pos'] == False:
           time.sleep(0.01)

        # Initialise robot class

        Init_pos =d['pos']
        Ts = 0.1
        kp = 3    #0.15   #0.5
        ka = 35  #0.4    #0.8
        kb = -8   #-0.07  #-0.2
        vTOm=31.5 #30.30
        wTOm=(200*180)/(80*math.pi) #130.5 #

        global_path = d['path']
        thym = Robot(global_path,Init_pos,Ts, kp,ka,kb,vTOm,wTOm)

        # Initialise Filtering class
        Rvel = np.array([[1.53, 0.], [0.,1.53]])
        Hvel = np.array([[0.,0.,0.,1.,0.],[0.,0.,0.,0.,1.]])
        Rcam = np.array([[0.000001,0.,0.],[0.,0.000001,0.],[0.,0.,0.000001]])
        Hcam = np.array([[1.,0.,0.,0.,0.],[0.,1.,0.,0.,0.],[0.,0.,1.,0.,0.]])

        filter = Filtering(Rvel, Rcam, thym, Hvel, Hcam,Ts)
        go=1

        while go:
            tps1 = time.monotonic()
            print("globpath: " + d['pos'])

             # GET THE GLOBAL PATH WITH THE CAMERA AND THE GLOBAL PATH CLASS : 

            # global_path = d['path']
            thym.global_path = global_path
            pos_cam = d['pos'] if d['pos'] else np.array[[0],[0]]

            rt = time.process_time() - t0
            t0 = time.process_time()
            if int(round(time.process_time(),0)) > t:
                t = int(round(time.process_time(),0))
                nd = dict(d)
                if self.verbose:
                    print("control period : " + str(rt))
                    print("vision period : "+d["vtime"])
                    print("position of robot : "+str(nd['pos']))



            thym.compute_state_equation(Ts)
            thym.compute_Pos()
            thym.check()
            thym.compute_input()
            thym.run_on_thymio(self.th)

            #[give : x,y,theta,vr,vl] to the filter : 
            x=float(thym.Pos[0])
            y=float(thym.Pos[1])
            theta=float(thym.Pos[2])

            vL=int(thym.ML) if thym.ML<=500 else thym.ML - 2** 16 
            vR=int(thym.MR) if thym.MR<=500 else thym.MR - 2** 16 
    
            vect=np.array([[x],[y],[theta],[vL],[vR]])

            time.sleep(0.1)
    
            # get the measurements from the camera : 

            # get our pos with the filter
            filter.compute_kalman(pos_cam,vect,self.th,Ts,False)

            thym.compute_pba(verbose=True)

            tps2 = time.monotonic()
            Ts=tps2-tps1

            if thym.p<3 and thym.node==len(thym.global_path)-2:
                self.th.set_var("motor.left.target", 0)
                self.th.set_var("motor.right.target", 0)
                print('FININSH!!!!')
                tfinal=time.monotonic()
                go = 0

            

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
    d['goal'] = False
    d['vtime'] = "0"

    cmptVis = ComputeVision(False)

    cmptVis.run(d)
    ctrl.run(d)
    cmptVis.join()
    ctrl.join()
