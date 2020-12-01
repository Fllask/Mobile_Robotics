
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

global_path = [(100,100),(75,25),(50,50)]

# Initialise robot class

Init_pos = np.array([100.,100.,0])
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



pos_cam=False
while thym.go:
    tps1 = time.monotonic()


    if thym.state =='ASTOLFI' : 
        thym.ASTOLFI(th,Ts, filter,pos_cam)
    elif thym.state == 'TURN' :
        thym.TURN(th,Ts)
    elif thym.state == 'LOCAL' :
        thym.LOCAL(th,Ts)
    elif thym.state == 'INIT' :
        thym.INIT(global_path,Init_pos)
    
    tps2 = time.monotonic()
    Ts=tps2-tps1

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


