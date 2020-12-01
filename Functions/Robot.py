from Thymio import Thymio
import os
import sys
import time
import numpy as np
import math as m
import pandas as pd
import math as m
import serial
from Utilities import Utilities
ut = Utilities()

""" Developped by: Thomas """
class Robot:
    """ Handles the control of the robot """
    
    #init
    def __init__(self,global_path,InitPos,Ts,kp,ka,kb,vTOm,wTOm):
        self.ka=ka                         #the three constant for the astolfi controller
        self.kb=kb
        self.kp=kp
        self.global_path=global_path       # global path in tuple
        self.Pos=InitPos                   # x,y and theta
        self.node=0
        
        self.go=1
        self.a=None                        # coordinate of the robot for the astolfi controller
        self.b=None
        self.p=None
        self.vTOm=vTOm                     # constant to convert velocity with value to give to the left and right motor
        self.wTOm=wTOm                     # constant to convert angular velocity with value to give to the left and right motor
        self.compute_pba()                 # compute values for rho alpha and beta
        
        self.u=np.array([0.0,0.])          # [v;w] speed and angular velocity
        self.ML=0                          # value to put in the left part of the motor
        self.MR=0                          # value to put on the right wheel motor
        self.state='INIT'                  # state of the robot [INIT,ASTOLFI,TURN,LOCAL,WAIT,SUCCES]

        # for local avoidance : 
        #self.avoidobstacle=0
        self.locstate=0                    #0->turn left (right), 1-> go straight, 2-> turn right (left) 
        self.turn=0                        #0 if avoid obstacle by the left or 1 if avoid obstacle by the right
        self.cnt=1                         # counter to repeat the loop for going straight in local avoidance 
        self.idx_sensor=(1,0)              # index of the sensor use for local avoidance to know which sensor are important depending if we are turning left or right
    
    
    def compute_pba(self,verbose = False):
        
        self.a=-self.Pos[2]+ut.compute_angle(self.Pos[0:2],self.global_path[self.node+1])
        self.p=ut.compute_distance(self.Pos[0:2],self.global_path[self.node+1])
        self.bref=-ut.compute_angle(self.global_path[self.node],self.global_path[self.node+1])
        self.b=-self.Pos[2]-self.bref-self.a 
        if verbose:
            print("a : " + str(self.a) + " , b : " + str(self.b) + " , p : " + str(self.p)) 
        return self.b

    # assume alpha(0)is between -pi/2 and pi/2 and stays between those two values
    def astofli_controller(self,p,a,b):
        p_dot=-self.kp*m.cos(a)
        a_dot=(self.kp*m.sin(a)-self.ka*a-self.kb*b)/p
        b_dot=(-self.kp*m.sin(a))/p
        return p_dot,a_dot,b_dot
    
    def compute_rotation(self,Ts):
        if self.a>0:
            w=0.3
            self.Pos[2]=self.Pos[2]+w*Ts
            self.a=self.a-w*Ts
        else:
            w=-0.3
            self.Pos[2]=self.Pos[2]+w*Ts
            self.a=self.a-w*Ts
        self.u[0]=0
        self.u[1]=w
        if abs(self.a)< 0.1:
            self.state='ASTOLFI'
        
        return self.a




    def compute_state_equation(self,Ts):
        #use runge Kutta 2 for state space equation


        [p_dot1,a_dot1,b_dot1]=self.astofli_controller(self.p,self.a,self.b)

        p1=self.p+Ts*p_dot1
        a1=self.a+Ts*a_dot1
        b1=self.b+Ts*b_dot1

        [p_dot2,a_dot2,b_dot2]=self.astofli_controller(p1,a1,b1)

        self.p=self.p+Ts/2.*p_dot1+Ts/2.*p_dot2
        self.a=self.a+Ts/2.*a_dot1+Ts/2.*a_dot2
        self.b=self.b+Ts/2.*b_dot1+Ts/2.*b_dot2
        

        self.u[0]=self.kp
        self.u[1]=(self.ka*self.a+self.kb*(self.b))/self.p

    def compute_input(self):
        #function convert the speed and angular speed to the values we need to give the robot

        # u[0] is the speed of the robot in cm/s
        # u[1] is the angular speed of the robot in cm/s

        

        vM=self.u[0]*self.vTOm
        wM=self.u[1]*self.wTOm
        
        ML=vM+wM
        MR=vM-wM
        ML = ML if ML >= 0 else 2 ** 16 - 1 + ML
        ML = int(ML)
        MR = MR if MR >= 0 else 2 ** 16 - 1 + MR
        MR = int(MR)

        self.ML=ML
        self.MR=MR+3

        

    # give values to the motors of the thymio
    def run_on_thymio(self,th):
        th.set_var("motor.left.target", self.ML)
        th.set_var("motor.right.target", self.MR)
        
        return self.ML

    # convert rho alpha and beta with x y and theta
    def compute_Pos(self):
        nextpos=self.global_path[self.node+1]
        self.Pos[0]=float(nextpos[0]-self.p*m.cos(self.b+self.bref))
        self.Pos[1]=float(nextpos[1]+self.p*m.sin(self.b+self.bref))
        self.Pos[2]= float(-(self.b+self.bref+self.a))
        return self.Pos[0]

    def check(self):
        if self.p<5 and self.node<(len(self.global_path)-2):
            #compute a new alpha,beta,gamma
            self.node=self.node+1
            self.compute_pba()
        return self.node

    # FUNCTION USE FOR THE LOCAL AVOIDANCE : 

    #function used to go straight
    def compute_straight_local(self,Ts,v):
        self.Pos[0]=self.Pos[0]+m.cos(self.Pos[2])*v*Ts
        self.Pos[1]=self.Pos[1]+m.sin(self.Pos[2])*v*Ts
        return self.u[0]
    #function used to turn left or right
    def compute_rot_local(self,Ts,w):
        self.Pos[2]=self.Pos[2]+w*Ts
        return self.Pos[2]

    #function use to check if we have a local obstacle
    def check_localobstacle(self,th) :
        sensor= np.array(th["prox.horizontal"])
        print('sensor',sensor)
        if sum(sensor[0:4])>2000: # threshold a modifié 
            self.state='LOCAL'
            print('state',self.state)
            right=sensor[4]+sensor[3]   #sensors at the right  
            left=sensor[0]+sensor[1]    #sensors at the left 
            if right>left:              #turn right if it feels the object on the left
                self.turn=0
                self.idx_sensor=(3,4)
            else:           #turn left if it feels the object on the right 
                self.turn=1
                self.idx_sensor=(1,0)
        return self.state

   
    # check if we are still in localstate 0 or if we can go to localstate 1
    def checkstate0(self,th):
        sensor= np.array(th["prox.horizontal"])
        if sensor[self.idx_sensor[1]]<1 and sensor[self.idx_sensor[0]]<1:
            self.locstate=1
            self.cnt=1
            #print("state1")
        return self.locstate

     # check if we are still in localstate 2 or if we can go to localstate 0
    def checkstate2(self,th):
        sensor= np.array(th["prox.horizontal"])
        if sensor[self.idx_sensor[1]]>1000:
            self.locstate=0
            #print("state2to3")
        return self.locstate

    # check if we still need to be in local avoidance or if we can go in global avoidance
    def checkout(self,th):
        sensor= np.array(th["prox.horizontal"])
        if sensor[self.idx_sensor[1]]<1:
            angle=ut.compute_angle(self.Pos[0:2],self.global_path[self.node+1])

            if abs(self.Pos[2]-angle)<0.02:
                self.state='INIT'
                self.u[0]=0
                self.u[1]=0

                th.set_var('motor.left.target',0)
                th.set_var('motor.right.target',0)
        return self.state

    def INIT(self,global_path, pos_init) : 
        
        self.state = 'INIT'
        ''' the robot stays in INIT state until it gets a global path and an initial position
        global path : give false if no global path else it gives the global path in a tuple
        pos_init: the initial position (x,y,theta)'''

        if global_path is not False :
            self.global_path = global_path
        else :
            return self.state
        if pos_init is not False :
            self.pos_init = pos_init
        else : 
            return self.state

        self.state = 'ASTOLFI'
        return self.state
    
    def get_states(self) :
        ''' funtion to give the state of the robot to the kalman filter '''

        x=float(self.Pos[0])
        y=float(self.Pos[1])
        theta=float(self.Pos[2])

        vL=int(self.ML) if self.ML<=500 else self.ML - 2** 16 
        vR=int(self.MR) if self.MR<=500 else self.MR - 2** 16 
    
        vect=np.array([[x],[y],[theta],[vL],[vR]])

        return vect

    def ASTOLFI(self,th,Ts,filter,pos_cam):
        ''' Astolfi controller with constant speed is used to control the robot when the angle alpha is between -pi/2 and pi/2
             th : serial link to the thymio
             Ts : time of one iteration of the loop while (we recompute every Ts)
             filter: a kalman filter is used using the measurement of the left and right motor and the measurement of the camera
             pos_cam: measurement of the camera'''
        
        # check if we detect a local obstacle
        self.check_localobstacle(th)

        if self.state == 'LOCAL' :
            return

        # converting x,y and theta in rho, beta and alpha (Astolfi controller)
        self.compute_pba()

        if abs(self.a)>m.pi/2:
            # calculate rho, beta and alpha at time t+1(Astolfi controller)
            self.state='TURN'
            return self.state
        else :
            # compute rho, alpha and beta at time t+ts
            self.compute_state_equation(Ts)
            # convert rho, beta and alpha in x y and theta (need those parameters for the filter)
            self.compute_Pos()
            # check if we are close to the next point in the global path and change the next goal in the astolfi controller if it is the case
            self.check()
            # calculate the velocity and angular velocity and the value we need to give to the left and right motor
            self.compute_input()

            # give the value of the motor to the thymio 
            self.run_on_thymio(th)

            #[give : x,y,theta,vr,vl] to the filter : 
            vect = self.get_states()

            # sleep 0.1 second :
            time.sleep(0.1)

            # check if we have a valid data for the measurement of the position in the camera
            update_cam = False if pos_cam is False else True

            # get our pos with the filter
            filter.compute_kalman(pos_cam,vect,th,Ts,update_cam)
            return self.state
        

    def TURN(self,th,Ts):

        '''if abs(alpha)>pi/2 we can't use astolfi and we first need to rotate the robot on itslef. We make it turn on  itself until alpha is close to 0
           th : serial link to the thymio
           Ts : time of one iteration of the loop while (we recompute every Ts)'''

        self.compute_rotation(Ts)
        # calculate the velocity and angular velocity and the value we need to give to the left and right motor
        self.compute_input()

        # give the value of the motor to the thymio 
        self.run_on_thymio(th)

        # sleep 0.1 second :
        time.sleep(0.1)

        return self.state

    def LOCAL(self,th,Ts):
        ''' we get around the local obstacle  until we we don't detect any obstacle in front of the robot when the robot is oriented in the direction of the goal
        the robot will enter in wait mode until we recompute a global path 
        th : serial link to the thymio
        Ts : time of one iteration of the loop while (we recompute every Ts)
        '''

        #turn left if the obstacle is on the right or turn right is the obstacle is on the left until the sensors don't sense the obstacle anymore
        if self.locstate==0: 
            w=0.3
            if self.turn==0: #turn left
                self.u[0]=0
                self.u[1]=-w
            else:
                self.u[0]=0  #turn right
                self.u[1]=w
            self.compute_rot_local(Ts,self.u[1])
            self.checkstate0(th)
            self.cnt=1

        # go straight during 20 loop :
        if self.locstate==1:
            self.cnt=self.cnt+1
            self.u[0]=3 # go straight
            self.u[1]=0
            # go straight
            self.compute_straight_local(Ts,self.u[0])
            if self.cnt==20:
                self.locstate=2
           
        # turn right until it sense the obstacle to stay close to the obstacle 
        if self.locstate==2:
            w=0.3
            if self.turn==0: #turn right
                self.u[0]=0
                self.u[1]=w
            else:
                self.u[0]=0  #turn left
                self.u[1]=-w
            #compute rotation
            self.compute_rot_local(Ts,self.u[1])

            #check if we still need to turn right or if we can turn left
            self.checkstate2(th)
            #check if we are still in local avoidance or if we can go back in global navigation
            self.checkout(th)

        # calculate the velocity and angular velocity and the value we need to give to the left and right motor
        self.compute_input()

        # give the value of the motor to the thymio 
        self.run_on_thymio(th)

        # sleep 0.1 second :
        time.sleep(0.1)

        return self.state