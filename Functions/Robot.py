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
        self.ka=ka
        self.kb=kb
        self.kp=kp
        self.global_path=global_path
        self.Pos=InitPos   # x,y and theta
        self.node=0
        self.astolfi=1
        self.go=1
        self.a=None
        self.b=None
        self.p=None
        self.vTOm=vTOm
        self.wTOm=wTOm
        self.compute_pba()
        
        self.u=np.array([0.0,0.0])          # [v;w] speed and angular velocity
        self.ML=0                           # value to put in the left part of the motor
        self.MR=0                           # value to put on the right wheel motor
        self.state=0                        # state of the robot 
    
    
    
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
            self.astolfi=1
        
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
        # astofli controller proportional to the speed: take a point 10 cm in front 

        vM=self.u[0]*self.vTOm
        wM=self.u[1]*self.wTOm
        
        
        if abs(wM)>500:
            print('ERROR wM')
        if abs(self.a)>m.pi/2:
            print('ERROR alpha')
        
        ML=vM+wM
        MR=vM-wM
        ML = ML if ML >= 0 else 2 ** 16 - 1 + ML
        ML = int(ML)
        MR = MR if MR >= 0 else 2 ** 16 - 1 + MR
        MR = int(MR)

        self.ML=ML
        self.MR=MR+3

        return self.ML

    def run_on_thymio(self,th):
        th.set_var("motor.left.target", self.ML)
        th.set_var("motor.right.target", self.MR)
        #print('ML\n',self.ML)
        #print('MR\n',self.MR)
        
        return self.ML

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



    def thymio(self,Ts,th):
        # converting x,y and theta in rho, beta and alpha (Astolfi controller)
        self.compute_pba()
        
        if self.astolfi==1 : 
            if abs(self.a)>m.pi/2:
                # calculate rho, beta and alpha at time t+1(Astolfi controller)
                self.astolfi=0
                print('test works')
            
        if self.astolfi==1:
            self.compute_state_equation(Ts)
            # convert rho, beta and alpha in x y and theta (need those parameters for the filter)
            self.compute_Pos()
            # check if we are close to the next point in the global path and change the next goal in the astolfi controller if it is the case
            self.check()
            

        else : 
            # if abs(alpha)>pi/2 we can't use astolfi and we first need to rotate the robot on itslef.
            self.compute_rotation(Ts)
            

        # calculate the velocity and angular velocity and the value we need to give to the left and right motor
        self.compute_input()
        # give the value of the motor to the thymio 
        self.run_on_thymio(th)
        return self.Pos[0]
