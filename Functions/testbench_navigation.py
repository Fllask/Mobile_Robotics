
from Functions.Robot import Robot
from Functions.Filtering import Filtering
from Functions.Thymio import Thymio
import os
import sys
import numpy as np
import math as m
import time

path = r'Functions\values.txt'

with open(path,"w") as f :
        f.writelines(f'time\t' +'Vl_filter\t' + 'Vr_filter\t\n')

#
th = Thymio.serial(port="COM3", refreshing_rate=0.1)

time.sleep(3) # To make sure the Thymio has had time to connect

# GET THE GLOBAL PATH WITH THE CAMERA AND THE GLOBAL PATH CLASS : 

global_path = [(0,0),(60,0)]

# Initialise robot class

Init_pos = np.array([0.,0.,0.])
Ts = 0.1
kp = 3    #0.15   #0.5
ka = 35  #0.4    #0.8
kb = -8   #-0.07  #-0.2

vTOm=31.5 #30.30
wTOm=(200*180)/(80*m.pi) #130.5 #

thym = Robot(global_path,Init_pos,Ts, kp,ka,kb,vTOm,wTOm)

# Initialise Filtering class
Rvel = np.array([[1.53, 0.], [0.,1.53]])
Hvel = np.array([[0.,0.,0.,1.,0.],[0.,0.,0.,0.,1.]])
Rcam = np.array([[0.000001,0.,0.],[0.,0.000001,0.],[0.,0.,0.000001]])
Hcam = np.array([[1.,0.,0.,0.,0.],[0.,1.,0.,0.,0.],[0.,0.,1.,0.,0.]])

filter = Filtering(Rvel, Rcam, thym, Hvel, Hcam,Ts)

go=1


# Begin testbench


Time=0
tinit=time.monotonic()

tstart = time.time()
while go:
    tps1 = time.monotonic()
    if abs(thym.a)<m.pi/2:
        thym.compute_state_equation(Ts)
    else:
        thym.compute_rotation(Ts)

    thym.compute_Pos()
    thym.check()
    thym.compute_input()
    thym.run_on_thymio(th)
    
    #[give : x,y,theta,vr,vl] to the filter : 
    x=float(thym.Pos[0])
    y=float(thym.Pos[1])
    theta=float(thym.Pos[2])

    

    vL=int(thym.ML) if thym.ML<=500 else thym.ML - 2** 16 
    vR=int(thym.MR) if thym.MR<=500 else thym.MR - 2** 16 
    
    vect=np.array([[x],[y],[theta],[vL],[vR]])

    telapsed = time.time() - tstart

    Xcam = 3.*telapsed

    pos_cam = np.array([[Xcam],[0],[0]])
    # Print current state (no filter)
    #print('x,y,theta, Vl,Vr\n', vect)

    # sleep 0.1 second :
   
    time.sleep(0.1)
    
    # get the measurements from the camera : 

    # get our pos with the filter
    X_filter=filter.kalman(pos_cam,vect,th,Ts,True)
    filter.compute_Q(Ts, 6.15)
    thym.Pos=X_filter[0:3]
    thym.ML=X_filter[3]
    thym.MR=X_filter[4]

    #Vl_filter = X_filter[3]
    #Vr_filter = X_filter[4]

    #theta_filter = X_filter[2]

    # with our new pose recompute p b and a (state of the astofi system)
    thym.compute_pba()
    tps2 = time.monotonic()
    Ts=tps2-tps1

    


    if thym.p<3 and thym.node==len(thym.global_path)-2:
        th.set_var("motor.left.target", 0)
        th.set_var("motor.right.target", 0)
        print('FININSH!!!!')
        tfinal=time.monotonic()
        print('Final time in the robot\n',tfinal-tinit)
        print('time of our state space equation\n',Time)
        go=0

    #with open(path,"a") as f :
        #f.write(f'' + str(telapsed) + '\t')
        #f.write(f'' + str(vL) + '\t')
        #f.write(f'' + str(vR) + '\n')


