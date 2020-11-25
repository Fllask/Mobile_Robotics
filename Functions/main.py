#!/Library/Frameworks/Python.framework/Versions/3.8/bin/python3.8
#the line above makes it more nicely executable on my mac, PATH may differ on different machines

# general includes (os stuff so that we can do multiprocesses)
import os #to access os parameters (Process Ids mostly)
from multiprocessing import * # I use the multiprocessing library for parallelism ==> https://docs.python.org/3.8/library/multiprocessing.html
import time #to time routines and have some idea of what is slow
import cv2
print('OpenCL available:', cv2.ocl.haveOpenCL())
import numpy as np
import math

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

# main function, root of all the program
if __name__ == '__main__':

    t0 = time.process_time()

    input_path = '../sample_pictures/test_set_2/03.jpg'
    img = v.get_image(input_path)

    print("get_image : "+str(time.process_time()-t0))
    t0 = t0 = time.process_time()

    imgprep = v.preprocess(img)

    print("preprocess : "+str(time.process_time()-t0))
    t0 = t0 = time.process_time()

    vis = v.Vision(imgprep)

    print("Vision : "+str(time.process_time()-t0))
    t0 = t0 = time.process_time()

    print("Display : "+str(time.process_time()-t0))

    rob = np.array([500,500,math.pi/2])

    img_real = cv2.warpPerspective(img, vis.trans, (1000,1000))
    obstacles = vis.getMap(downscale = False)
    cv2.drawContours(img_real, obstacles, -1, (0,255,0), 3)
    for p in obstacles:
        for corner in p:
            cv2.circle(img_real, tuple(corner.reshape(2)), 5, (255,255,0), thickness=1, lineType=8, shift=0)
    pt1 = (int(rob[0]), int(rob[1]))
    pt2 = (int(rob[0]+math.cos(rob[2])*100), int(rob[1]+math.sin(rob[2])*100))
    cv2.line(img_real,pt1,pt2,(128,128,0),thickness=3)
    cv2.circle(img_real,pt1,10,(128,128,0),thickness = 4)
    cv2.namedWindow('map',cv2.WINDOW_NORMAL)
    cv2.resizeWindow('map', 600,600)
    cv2.imshow('map', img_real)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    #
    # 


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
        

