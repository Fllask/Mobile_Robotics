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
import pickle

from reprint import output ## allows for nice multiline dynamic printing

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
    def __init__(self,verbose, fileinput):
        self.verbose = verbose
        self.fileinput = fileinput

        self.kalTrace = []
        self.visTrace = []


    """ Starts the vision process """
    def run(self,d):
        self.mainLoopProcess = Process(target=self.mainLoop, args=(d,))
        self.mainLoopProcess.start()
    
    def join(self):
        self.mainLoopProcess.join()

   
    """ Displays processed data """
    def display(self,d, text = False, trace = False):
        scalef = 5.     # scalefactor (the pixel grid is 5x larger than the map) 
        ilength = 8.    # length of the indicator of robot position
        irad = 30       # radius of the robot marker
        lineW = 2       # linewidth for the plot

        # colors of the different display elements
        visColor = (0,0,255)
        kalColor = (0,255,255)
        mapColor = (0,255,0)
        goalColor = (255,0,0)
        pathColor = (0,0,0)
        textColor = (0,0,0)

        # saving trace of the robot (vision position)
        if not isinstance(d['visPos'], bool):
            self.kalTrace.append( (d['visPos'][0],d['visPos'][1]) )
        # saving trace of the robot (kalman position)   
        if not isinstance(d['fltPos'], bool):
            self.visTrace.append( (d['fltPos'][0],d['fltPos'][1]) )

        #projecting the image
        tr_img =  cv2.cvtColor(self.vis.frame,cv2.COLOR_HSV2BGR)

        font = cv2.FONT_HERSHEY_SIMPLEX # font of the text

        #########################################################################
        #                                                                       #
        #                  Displaying robot position and trace                  #
        #                                                                       #
        #########################################################################

        #plotting the obstacles detected
        if not isinstance(self.vis.getMap(downscale=False),(bool)):
            cv2.drawContours(tr_img, self.vis.getMap(downscale=False), -1, mapColor, 2)
        
        #plotting the gobal planned navigation path
        if not isinstance(self.path,bool):
            path = self.path
            for i in range(1,len(path)) :
                cv2.line(tr_img,(int(path[i][0]*scalef),int(path[i][1]*scalef)),(int(path[i-1][0]*scalef),int(path[i-1][1]*scalef)),pathColor ,thickness=2)

        #plotting the robot's trace
        if trace:
            for i in range(1,len(self.kalTrace)):
                seg = (self.kalTrace[i-1],self.kalTrace[i])
                if not ( isinstance(seg[0][0],bool) or isinstance(seg[1][0],bool) ):
                    cv2.line(tr_img,(int(seg[0][0]*scalef),int(seg[0][1]*scalef)),(int(seg[1][0]*scalef),int(seg[1][1]*scalef)),kalColor,thickness=2)

            for i in range(1,len(self.visTrace)):
                seg = (self.visTrace[i-1],self.visTrace[i])
                if not ( isinstance(seg[0][0],bool) or isinstance(seg[1][0],bool) ):
                    cv2.line(tr_img,(int(seg[0][0]*scalef),int(seg[0][1]*scalef)),(int(seg[1][0]*scalef),int(seg[1][1]*scalef)),visColor,thickness=2)

        fltPos = d['fltPos']
        if not isinstance(fltPos,bool):
            pt1 = (int(fltPos[0]*scalef), int(fltPos[1]*scalef))
            pt2 = (int(fltPos[0]*scalef+math.cos(fltPos[2])*scalef*ilength), int(fltPos[1]*5+math.sin(fltPos[2])*scalef*ilength))

            cv2.circle(tr_img,(int(fltPos[0]*scalef),int(fltPos[1]*scalef)),irad,kalColor,thickness=2)
            cv2.line(tr_img,pt1,pt2,kalColor,thickness=lineW)
            
        ## plotting the robot's position
        if not isinstance(self.rob,bool):
            pt1 = (int(self.rob[0]*scalef), int(self.rob[1]*scalef))
            pt2 = (int(self.rob[0]*scalef+math.cos(self.rbt_pos[2])*scalef*ilength), int(self.rob[1]*5+math.sin(self.rbt_pos[2])*scalef*ilength))

            cv2.circle(tr_img,(int(self.rbt_pos[0]*scalef),int(self.rbt_pos[1]*scalef)),irad,visColor,thickness=2)
            cv2.line(tr_img,pt1,pt2,visColor,thickness=lineW)
            
            if text:
                tr_img = cv2.putText(tr_img, 'rbt : ' + str(self.rbt_pos), (int(self.rbt_pos[0]*scalef),int(self.rbt_pos[1]*scalef)), font,  0.5, textColor, 1, cv2.LINE_AA) 

        ## plotting the goal
        if not isinstance(self.stop,bool):
            cv2.circle(tr_img,(int(self.stop[0]*scalef),int(self.stop[1]*scalef)),irad,goalColor,thickness=lineW)
            if text:
                tr_img = cv2.putText(tr_img, 'goal : ' + str(self.stop), (int(self.stop[0]*scalef),int(self.stop[1]*scalef)), font,  0.5, textColor, 1, cv2.LINE_AA) 

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
            cap = "../sample_pictures/test_border.jpg"
        else:
            cap = cv2.VideoCapture(1)
        

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
            self.vis = v.Vision(frame, "ANDROID FLASK",verbose=flag,setmanually = True,setextval = True)
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
        
        self.g = Global(self.obstacles,False,self.stop,margin=1.5)
        
            
                
        if self.verbose:
            print("Initial Path Planning Time : "+str(time.process_time()-t0))


        runFlag = True
        d['started'] = True
        while runFlag: 

            #########################################################################
            #                                                                       #
            #                        Vision Loop of the robot                       #
            #                                                                       #
            #########################################################################
            
             #we time each loop to get an idea of performance
            t0 = time.process_time()

            # loading new image
            ret, self.img = self.loadImage(cap)
            self.vis.setframe(self.img) 
            
            ## getting robot coordinates and updating position for robot control loop
            self.rbt_pos, self.pos_valid = self.vis.returnDynamicCoordinates() 
            if not isinstance(self.rbt_pos,bool):
                self.rob = tuple(self.rbt_pos[0:2])
            else:
                self.rob = False
            d['visPos'] = self.rbt_pos

            ## displaying whatever was computed
            disp = self.display(d,text = False , trace = True)
            cv2.imshow('frame',disp)
            key = cv2.waitKey(1) 

            # detect keypress for graceful shutdown
            if cv2.waitKey(1) & 0xFF == ord('q'):
                d['running'] = False

            # wait until we have start, finish and obstacles to compute path
            if not (isinstance(self.stop, bool) or isinstance(self.rbt_pos, bool) or isinstance(self.obstacles, bool) or d['pathComputed']):
                self.g.start = self.rob
                self.path = self.g.returnPath(self.obstacles,self.rob,self.stop)
                d['path'] = self.path
                d['pathComputed'] = True
            
            d['vtime']=str(time.process_time()-t0)
            runFlag = d['running']


""" 
    RobotControl:
    Handles the control of the robot as a process (fast loop)
    @autor: Titou
"""
class RobotControl():
    """ Constructor of the RobotControl class """
    def __init__(self,verbose,address,norobot,save = False):
        self.verbose = True
        self.address = address
        self.norobot = norobot
        self.history = []
        self.save = save

    """ Starts the control process """
    def run(self,d):
        self.mainLoopProcess = Process(target=self.mainLoop, args=(d,))
        self.mainLoopProcess.start()

    def join(self):
        self.mainLoopProcess.join()
    
    def copyProxyDict(self,d):
        retDict = {}
        for key in d.keys():
            retDict[key] = d[key]

        return retDict

    def saveHistory(self):
        if self.verbose:
            print("-> Saving run history as : " + str(self.save))
        pickle.dump(self.history, open(self.save,"wb"))

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
        kp = 3.  #2 #3    #0.15   #0.5
        ka = 35. #22 #35  #0.4    #0.8
        kb = -8. #-4 #-8   #-0.07  #-0.2
        vTOm=31.5 #30.30
        wTOm=(200*180)/(80*math.pi) #130.5 #
        thym = Robot(False,False,Ts, kp,ka,kb,vTOm,wTOm)

        # Initialise Filtering class

        Rvel = np.array([[2., 0.], [0.,2.]])
        Hvel = np.array([[0.,0.,0.,1.,0.],[0.,0.,0.,0.,1.]])
        Rcam = np.array([[2.,0.,0.],[0.,2.,0.],[0.,0.,0.3]])
        Hcam = np.array([[1.,0.,0.,0.,0.],[0.,1.,0.,0.,0.],[0.,0.,1.,0.,0.]])

        filter = Filtering(Rvel, Rcam, thym, Hvel, Hcam,Ts)

        init = False
        go=1
        cnt = 0

        """ wait until vision is go"""
        while not d['started']:
            time.sleep(0.1)
        print("STARTING CONTROL, PRESS Q TO SHUTDOWN : \n")
        with output(output_type='dict') as output_lines:
            while go:

                # Some simulation variables
                tps1 = time.monotonic()
                cnt += 1

                #########################################################################
                # get every 1 second the position of the robot given by the camera      #
                # when it is possible if not possible set the updateWithCam bolean      #    
                #                           to False                                    # 
                #########################################################################
                pos_cam = np.array(d['visPos'],ndmin=2).T

                if not cnt%10 : 
                    update_cam = False if pos_cam is False or (pos_cam[0] == 0) else True
                else :
                    update_cam = False

                #########################################################################
                #                                                                       #
                #                           FSM of the Robot                            #
                #                                                                       #
                #########################################################################

                if not self.norobot:
                    if thym.state =='ASTOLFI' : 
                        thym.ASTOLFI(self.th,Ts, filter,pos_cam, update_cam)
                        if thym.state == 'INIT':
                            d['pathComputed'] = False
                            d['path'] = False
                    elif thym.state == 'TURN' :
                        thym.TURN(self.th,Ts, filter, pos_cam, update_cam)
                    elif thym.state == 'LOCAL' :
                        thym.LOCAL(self.th,Ts, filter, pos_cam, update_cam)
                        # Tell vision to recompute a path
                        if thym.state == 'INIT':
                            d['pathComputed'] = False
                            d['path'] = False
                    elif thym.state == 'INIT' :
                        thym.INIT(d['path'],d['visPos'])

                #########################################################################
                #                                                                       #
                #            Compute the time the control took, to adjust Ts            #
                #                                                                       #
                #########################################################################

                tps2 = time.monotonic()
                Ts=tps2-tps1
                d['fltPos'] = thym.Pos

                
                #########################################################################
                #                                                                       #
                #                  Check if the robot is at the goal                    #
                #                                                                       #
                #########################################################################

                if not self.norobot:
                    go = thym.FINISH(self.th, go)
                    tfinal=time.monotonic()                    

                rt = time.process_time() - t0
                t0 = time.process_time()

                precision = 1 # we update display and history 10x a second

                if round(time.process_time(),precision) > t: 
                    t = round(time.process_time(),precision)
                    """ Nice display for the robot control """
                    if self.verbose:
                        output_lines['HISTORY SAMPLES'] = len(self.history)
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
                    histPoint = self.copyProxyDict(d)
                    histPoint['time'] = time.process_time()
                    if not self.norobot:
                        histPoint['state'] = thym.state
                    else:
                        histPoint['state'] = "NO ROBOT"

                    self.history.append(histPoint)

                if d['running'] == False:
                    go = 0
        print("CONTROL STOPPED\n")

        if(self.save,str):
            self.saveHistory()

            

"""
    main function, root the program
    @autor: Titou
"""
if __name__ == '__main__':

    print('OpenCL available:', cv2.ocl.haveOpenCL())

    robotPort = "COM3"
    saveFile = "history.pkl"

    """ Parsing stdin """
    verbose = False
    fileinput = False
    norobot = False
    save = False
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
        """ f flag is for saving control history """
        if sys.argv[i] == "r":
            print("RUNNING WITH HISTORY")
            save = saveFile


    
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
    ctrl = RobotControl(verbose,robotPort,norobot,save)

    """ starting threads """
    cmptVis.run(d)
    ctrl.run(d)
    cmptVis.join()
    ctrl.join()

    print("Sucessful graceful exit")