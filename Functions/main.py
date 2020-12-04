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

from reprint import output ## allows for nice multiline dynamic printing

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
    def __init__(self,verbose, fileinput):
        self.verbose = verbose
        self.fileinput = fileinput


    """ Starts the vision process """
    def run(self,d):
        self.mainLoopProcess = Process(target=self.mainLoop, args=(d,))
        self.mainLoopProcess.start()
    
    def join(self):
        self.mainLoopProcess.join()

   
    """ Displays processed data """
    def display(self,d):
        scalef = 5.
        ilength = 8.
        irad = 30
        lineW = 2

        #projecting the image
        tr_img =  cv2.cvtColor(self.vis.frame,cv2.COLOR_HSV2BGR)

        font = cv2.FONT_HERSHEY_SIMPLEX 

        #plotting the obstacles detected
        if not isinstance(self.vis.getMap(downscale=False),(bool)):
            cv2.drawContours(tr_img, self.vis.getMap(downscale=False), -1, (0,255,0), 2)
        
        #plotting the gobal navigation path
        if not isinstance(self.path,bool):
            path = self.path
            for i in range(1,len(path)) :
                cv2.line(tr_img,(int(path[i][0]*scalef),int(path[i][1]*scalef)),(int(path[i-1][0]*scalef),int(path[i-1][1]*scalef)),(0,0,0),thickness=2)

        fltPos = d['fltPos']
        if not isinstance(fltPos,bool):
            pt1 = (int(fltPos[0]*scalef), int(fltPos[1]*scalef))
            pt2 = (int(fltPos[0]*scalef+math.cos(fltPos[2])*scalef*ilength), int(fltPos[1]*5+math.sin(fltPos[2])*scalef*ilength))

            cv2.circle(tr_img,(int(fltPos[0]*scalef),int(fltPos[1]*scalef)),irad,(0,255,255),thickness=2)
            cv2.line(tr_img,pt1,pt2,(0,255,255),thickness=lineW)

            # tr_img = cv2.putText(tr_img, 'rbt : ' + str(self.rbt_pos), (int(self.rbt_pos[0]*scalef),int(self.rbt_pos[1]*scalef)), font,  0.5, (0,0,0), 1, cv2.LINE_AA) 
        
        ## plotting the robot's position
        if not isinstance(self.rob,bool):
            pt1 = (int(self.rob[0]*scalef), int(self.rob[1]*scalef))
            pt2 = (int(self.rob[0]*scalef+math.cos(self.rbt_pos[2])*scalef*ilength), int(self.rob[1]*5+math.sin(self.rbt_pos[2])*scalef*ilength))

            cv2.circle(tr_img,(int(self.rbt_pos[0]*scalef),int(self.rbt_pos[1]*scalef)),irad,(0,0,255),thickness=2)
            cv2.line(tr_img,pt1,pt2,(0,0,255),thickness=lineW)

            tr_img = cv2.putText(tr_img, 'rbt : ' + str(self.rbt_pos), (int(self.rbt_pos[0]*scalef),int(self.rbt_pos[1]*scalef)), font,  0.5, (0,0,0), 1, cv2.LINE_AA) 
        ## plotting the goal
        if not isinstance(self.stop,bool):
            cv2.circle(tr_img,(int(self.stop[0]*scalef),int(self.stop[1]*scalef)),irad,(255,0,0),thickness=lineW)
            tr_img = cv2.putText(tr_img, 'goal : ' + str(self.stop), (int(self.stop[0]*scalef),int(self.stop[1]*scalef)), font,  0.5, (0,0,0), 1, cv2.LINE_AA) 

        return tr_img
        
    
    def loadImage(self, cap):
        if isinstance(cap, str):
            frame = cv2.imread(cap)
            return True, frame
        else:
            return cap.read()
        
    """ Main vision loop """
    def mainLoop(self,d):

        if self.verbose:
            print("Starting vision + global navigation main loop")

       
        #getting the camera input
        if self.fileinput:
            cap = "../sample_pictures/sample.jpg"
        else:
            cap = cv2.VideoCapture(0)
        

        #get the first frame to test
        ret, self.img = self.loadImage(cap)
        self.img =  cv2.resize(self.img,(624,416))

        if not ret:
            if self.verbose:
                print("frame droped")
                
         # initialization of the vision object
        t0 = time.process_time()
        
        initfailed = True
        flag = self.verbose
        if self.verbose:
            print("initializing vision")

        while(initfailed):
            ret,frame = self.loadImage(cap)
            self.vis = v.Vision(frame, "ANDROID FLASK",verbose=flag,setmanually = True)
            initfailed = self.vis.invalid
            flag = False
        
        if self.verbose:
            print("Vision Init Success")
        
        if self.verbose:
            print("Initial Mapping Time : " + str(time.process_time()-t0))
        
        #querying the aim coordinates
        self.stop, ret = self.vis.returnDynamicAim()
        if not isinstance(self.stop,bool):
            self.stop = tuple(self.stop)
        else:
            self.stop = False
        self.path = False



        # querying robot coordinates
        t0 = time.process_time()
        self.rbt_pos,ret = self.vis.returnDynamicCoordinates() ## getting robot coordinate
        
        if isinstance(self.rbt_pos,bool):
            self.rob = False
        else:
            self.rob = tuple(self.rbt_pos[0:2])
            
        d['visPos'] = self.rbt_pos
        
        if self.verbose:
            print("Initial Robot_Pos Estimation Time : "+str(time.process_time()-t0))
        
        #initializing the path planning module
        t0 = time.process_time()
        self.obstacles = self.vis.getMap()
        d['map'] = self.obstacles
        #self.pathComputed = False
        
        self.g = Global(self.obstacles,False,self.stop)
        
            
                
        if self.verbose:
            print("Initial Path Planning Time : "+str(time.process_time()-t0))


        runFlag = True
        while runFlag: 
            d['started'] = True
            t0 = time.process_time() #we time each loop to get an idea of performance
            # loading new image
            ret, self.img = self.loadImage(cap)
            self.vis.setframe(self.img) 
            
            ## getting robot coordinates
            self.rbt_pos, self.pos_valid = self.vis.returnDynamicCoordinates() 
            if not isinstance(self.rbt_pos,bool):
                self.rob = tuple(self.rbt_pos[0:2])
            else:
                self.rob = False
                
            d['visPos'] = self.rbt_pos

            ## displaying whatever was computed
            disp = self.display(d)
            cv2.imshow('frame',disp)
            key = cv2.waitKey(1) 

            if cv2.waitKey(1) & 0xFF == ord('q'):
                d['running'] = False


            if isinstance(self.stop, bool) or isinstance(self.rbt_pos, bool) or isinstance(self.obstacles, bool) or d['pathComputed']:#self.pathComputed
                if self.verbose:
                    pass
                    #   print("No Path Computed")
                #     print("stopB->"+str(isinstance(self.stop,bool)))
                #     print("rbt_posB->"+str(isinstance(self.rbt_pos,bool)))
                #     print("obstaclesB->"+str(isinstance(self.obstacles,bool)))
                #     print("pathComputed->"+str(self.pathComputed))
            else:
                self.g.start = self.rob
                self.path = self.g.returnPath(self.obstacles,self.rob,self.stop)
                # print(self.obstacles)
                d['path'] = self.path
                # print(self.path)
                #self.pathComputed = True
                d['pathComputed'] = True
            
            #if self.verbose:
                #print("Full Vision Loop : "+str(time.process_time()-t0))
            d['vtime']=str(time.process_time()-t0)
            runFlag = d['running']


""" 
    RobotControl:
    Handles the control of the robot as a process (fast loop)
    @autor: Titou
"""
class RobotControl():
    """ Constructor of the RobotControl class """
    def __init__(self,verbose,address,norobot):
        self.verbose = True
        self.address = address
        self.norobot = norobot

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

        if not self.norobot:
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

        Init_pos = False
        Ts = 0.1
        kp = 2 #3    #0.15   #0.5
        ka = 22 #35  #0.4    #0.8
        kb = -4 #-8   #-0.07  #-0.2
        vTOm=31.5 #30.30
        wTOm=(200*180)/(80*math.pi) #130.5 #
        thym = Robot(False,False,Ts, kp,ka,kb,vTOm,wTOm)

        # Initialise Filtering class

        Rvel = np.array([[100000000., 0.], [0.,10000000.]])
        Hvel = np.array([[0.,0.,0.,1.,0.],[0.,0.,0.,0.,1.]])
        Rcam = np.array([[2.,0.,0.],[0.,2.,0.],[0.,0.,0.3]])
        Hcam = np.array([[1.,0.,0.,0.,0.],[0.,1.,0.,0.,0.],[0.,0.,1.,0.,0.]])

        filter = Filtering(Rvel, Rcam, thym, Hvel, Hcam,Ts)

        init = False
        go=1
        cnt = 1

        """ wait until vision is go"""
        while not d['started']:
            time.sleep(0.1)
        print("STARTING CONTROL, PRESS Q TO SHUTDOWN : \n")
        with output(output_type='dict') as output_lines:
            while go:

                tps1 = time.monotonic()
                
                cnt += 1

                #########################################################################
                # get the position of the robot given by the camera when it is possible #
                #          if not possible set the updateWithCam bolean to False        # 
                #########################################################################

                if not cnt%10 : 
                    pos_cam = np.array(d['visPos'],ndmin=2).T
                else :
                    pos_cam = False


                #########################################################################
                #                                                                       #
                #                           FSM of the Robot                            #
                #                                                                       #
                #########################################################################

                if not self.norobot:
                    if thym.state =='ASTOLFI' : 
                        thym.ASTOLFI(self.th,Ts, filter,pos_cam)
                    elif thym.state == 'TURN' :
                        thym.TURN(self.th,Ts, filter, pos_cam)
                    elif thym.state == 'LOCAL' :
                        thym.LOCAL(self.th,Ts, filter, pos_cam)
                        if thym.state == 'INIT':
                            d['pathComputed'] = False
                            d['path'] = False
                    elif thym.state == 'INIT' :
                        # print('dpath',d['path'])
                        thym.INIT(d['path'],d['visPos'])



                tps2 = time.monotonic()
                Ts=tps2-tps1
                d['fltPos'] = thym.Pos

                if thym.p is not None :
                    if thym.p<5 and thym.node==len(thym.global_path)-2:
                        if not self.norobot:
                            self.th.set_var("motor.left.target", 0)
                            self.th.set_var("motor.right.target", 0)
                        print('FININSH!!!!')
                        tfinal=time.monotonic()
                        go = 0


                rt = time.process_time() - t0
                t0 = time.process_time()
                if int(round(time.process_time(),0)) > t:
                    t = int(round(time.process_time(),0))
                    """ Nice display for the robot control """
                    if self.verbose:
                        output_lines['CTRL PERIOD'] = str(rt)
                        output_lines['VISION PERIOD'] = str(d["vtime"])
                        output_lines['VISION POS'] = str(d['visPos'])
                        output_lines['KALMAN POS'] = str(d['fltPos'])
                        output_lines['GOAL'] = str(d['goal'])
                        output_lines['PATH'] = str(d['path'])
                        output_lines['RUNNING'] = str(d['running'])
                        if not self.norobot:
                            output_lines['STATE'] = str(thym.state)
                        else:
                            output_lines['STATE'] = " NO ROBOT"

                if d['running'] == False:
                    go = 0
        print("CONTROL STOPPED\n")

            

"""
    main function, root of all the program
    @autor: Titou
"""
if __name__ == '__main__':

    print('OpenCL available:', cv2.ocl.haveOpenCL())

    robotPort = "COM7"

    """ Parsing stdin """
    verbose = False
    fileinput = False
    norobot = False
    for i in range (1,len(sys.argv)):
        """ v flag is verbose """
        if sys.argv[i] == "v":
            print("RUNNING VERBOSE MODE")
            verbose = True
        """ w flag is webcamless vision debug """
        if sys.argv[i] == "w":
            print("RUNNING WEBCAMLESS DEBUG MODE")
            fileinput = True
        """ r flag is robotless debug """
        if sys.argv[i] == "r":
            print("RUNNING ROBOTLESS DEBUG MODE")
            norobot = True


    
    """ initializing memory manager"""
    manager = Manager()
    d = manager.dict()
    d['visPos'] = False
    d['fltPos'] = False
    d['path'] = False
    d['map'] = False
    d['goal'] = False
    d['pathComputed'] = False
    d['vtime'] = "0"
    d['started'] = False
    d['running'] = True

    """ initializing thread objects"""
    cmptVis = ComputeVision(False, fileinput)
    ctrl = RobotControl(verbose,robotPort,norobot)

    """ starting threads """
    cmptVis.run(d)
    ctrl.run(d)
    cmptVis.join()
    ctrl.join()

    print("Sucessful graceful exit")