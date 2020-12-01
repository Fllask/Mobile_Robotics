
from Robot import Robot
from Filtering import Filtering
from Thymio import Thymio
import os
import sys
import numpy as np
import math as m
import time

#path = r'Functions\values.txt'

#with open(path,"w") as f :
#        f.writelines(f'time\t' +'Vl_filter\t' + 'Vr_filter\t\n')

#
th = Thymio.serial(port="COM3", refreshing_rate=0.1)

time.sleep(3) # To make sure the Thymio has had time to connect

# GET THE GLOBAL PATH WITH THE CAMERA AND THE GLOBAL PATH CLASS : 

global_path = [(0,0),(200,0)]

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

thym.go=1


# Begin testbench




while thym.go:
    tps1 = time.monotonic()
    
    if thym.state=='ASTOLFI':
        thym.check_localobstacle(th)
    
    if thym.state=='LOCAL':
        thym.localavoidance(Ts,th)
    else:
        thym.thymio(Ts,th)
    
    print('POS',thym.Pos)
    # calculate the velocity and angular velocity and the value we need to give to the left and right motor
    thym.compute_input()
    # give the value of the motor to the thymio 
    thym.run_on_thymio(th)

    #[give : x,y,theta,vr,vl] to the filter : 
    x=float(thym.Pos[0])
    y=float(thym.Pos[1])
    theta=float(thym.Pos[2])
    vL=int(thym.ML) if thym.ML<=500 else thym.ML - 2** 16 
    vR=int(thym.MR) if thym.MR<=500 else thym.MR - 2** 16 
    
    vect=np.array([[x],[y],[theta],[vL],[vR]])


    # sleep 0.1 second :
   
    time.sleep(0.1)
    
    # get the measurements from the camera : 

    # get our pos with the filter
    if thym.state=='ASTOLFI': 
        pos_cam=[0]
        filter.compute_kalman(pos_cam,vect,th,Ts,False)
        print('filer')
    #Vl_filter = X_filter[3]
    #Vr_filter = X_filter[4]

    #theta_filter = X_filter[2]
    
    
    tps2 = time.monotonic()
    Ts=tps2-tps1

    #condition a enlever juste pour test
    if thym.state=='WAIT':
        thym.go=0

    # check if we arrive at the goal 
    if thym.p<3 and thym.node==len(thym.global_path)-2:
        th.set_var("motor.left.target", 0)
        th.set_var("motor.right.target", 0)
        print('FININSH!!!!')
        tfinal=time.monotonic()

        thym.go=0

    #with open(path,"a") as f :
        #f.write(f'' + str(telapsed) + '\t')
        #f.write(f'' + str(vL) + '\t')
        #f.write(f'' + str(vR) + '\n')


