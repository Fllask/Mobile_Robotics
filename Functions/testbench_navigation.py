
from Robot import Robot
from Filtering import Filtering
from Thymio import Thymio
import os
import sys
import numpy as np
import math as m
import time
#
th = Thymio.serial(port="\\.\COM3", refreshing_rate=0.1)

time.sleep(3) # To make sure the Thymio has had time to connect

# Initialise robot class

global_path = [(0,0),(60,31.5)]
Init_pos = np.array([0.,0.,0.])
Ts = 0.1
kp = 0.15
ka = 0.4
kb = -0.07

vTOm=31.25
wTOm=(200.0*180)/(80*m.pi)

thym = Robot(global_path,Init_pos,Ts, kp,ka,kb,vTOm,wTOm)

# Initialise Filtering class
Q = np.array([[6.15,0.,0.,0.615,0.615],[0,6.15,0.,0.615,0.615],[0.,0.,6.15,0.615,0.615],[0.,0.,0.,6.15,0.],[0.,0.,0.,0.,6.15]])
Rvel = np.array([[0.2, 0.], [0.,0.2]])
Hvel = np.array([[0.,0.,0.,1.,0.],[0.,0.,0.,0.,1.]])
Rcam = 0

filter = Filtering(Rvel, Rcam, Q, thym, Hvel, Ts)

go=1
# Begin testbench
while go:
    thym.compute_state_equation(Ts)
    thym.compute_Pos()



    thym.check()
    thym.compute_input()
    print('input',thym.ML)
    print('input,',thym.MR)
    thym.run_on_thymio(th)
    
    #print('POS',self.Pos)
    # ADD KALMAN FILTER : 
    #[give : x,y,theta,vr,vl]
    vect=np.array([thym.Pos[0],thym.Pos[1],thym.Pos[2],thym.u[0],thym.u[1]]).T
    x=float(thym.Pos[0])
    y=float(thym.Pos[1])
    theta=float(thym.Pos[2])
    vL=int(thym.ML)
    vR=int(thym.MR)

    vect=np.array([[x],[y],[theta],[vL],[vR]])
    print('vect',vect)


    time.sleep(Ts)

    X_filter=filter.kalman(0.0,vect,th)
    filter.compute_Q(Ts, 6.15)
    #print('Pos_no_filter\n',thym.Pos)
    thym.Pos=X_filter[0:3]
    print('Pos_filter\n',thym.Pos)
    thym.ML=X_filter[3]
    thym.MR=X_filter[4]

    thym.compute_pba()

    if thym.p<1 and thym.node==len(thym.global_path)-2:
        th.set_var("motor.left.target", 0)
        th.set_var("motor.right.target", 0)
        print('FININSH!!!!')
        go=0


