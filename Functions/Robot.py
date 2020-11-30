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
    
    
    
    def compute_pba(self):
            
        #print(self.node)
              
        #print('pos1',self.global_path[(self.node)])
        #print('pos2',self.global_path[(self.node+1)])
        #print('self global path',self.global_path)
        self.a=-self.Pos[2]+ut.compute_angle(self.Pos[0:2],self.global_path[self.node+1])
        #print('POS',self.Pos[0:2])
        self.p=ut.compute_distance(self.Pos[0:2],self.global_path[self.node+1])
        self.bref=-ut.compute_angle(self.global_path[self.node],self.global_path[self.node+1])

        self.b=-self.Pos[2]-self.bref-self.a 
        #print('self b',self.b)
        #print('self.a',self.a)
        #print('self.p',self.p)
        return self.b

    # assume alpha(0)is between -pi/2 and pi/2 and stays between those two values
    def astofli_controller(self,p,a,b):
        p_dot=-self.kp*m.cos(a)
        a_dot=(self.kp*m.sin(a)-self.ka*a-self.kb*b)/p
        b_dot=(-self.kp*m.sin(a))/p
        return p_dot,a_dot,b_dot
    
    def compute_rotation(self,Ts):
        if self.a>m.pi/2:
            w=0.3
            self.Pos[2]=self.Pos[2]+self.Pos[2]*w*Ts
            self.a=self.a-self.Pos[2]*w*Ts
        else:
            w=-0.3
            self.Pos[2]=self.Pos[2]+self.Pos[2]*w*Ts
            self.a=self.a-self.Pos[2]*w*Ts
        self.u[0]=0
        self.u[1]=w
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

        #print('p\n',self.p)
        #self.p=self.p*(1-Ts*self.kp*m.cos(self.a))
        #print('p after',self.p)
        #self.a=self.a+Ts*(self.kp*m.sin(self.a)-self.ka*self.a-self.kb*(self.b-self.bref))
        #self.b= self.b-Ts*(self.kp*m.sin(self.a))
        #print('alpha',self.a)
        #print('beta',self.b)




    
    def compute_input(self):
        # astofli controller proportional to the speed: take a point 10 cm in front 

        #print('v\n',self.u[0])
        #print('w\n',self.u[1])


        vM=self.u[0]*self.vTOm
        wM=self.u[1]*self.wTOm

        if abs(wM)>500:
            print('ERROR wM')
        if abs(self.a)>m.pi/2:
            print('ERROR alpha')
        
        ML=vM+wM
        MR=vM-wM
        #print('ML',self.ML)
        #print('MR',self.MR)
        ML = ML if ML >= 0 else 2 ** 16 + ML
        ML = int(ML)
        MR = MR if MR >= 0 else 2 ** 16 + MR
        MR = int(MR)

        self.ML=ML
        self.MR=MR+3

        return self.ML

    def run_on_thymio(self,th):
        th.set_var("motor.left.target", self.ML)
        th.set_var("motor.right.target", self.MR)
        print('ML\n',self.ML)
        print('MR\n',self.MR)
        
        return self.ML

    def compute_Pos(self):
        nextpos=self.global_path[self.node+1]
        self.Pos[0]=float(nextpos[0]-self.p*m.cos(self.b+self.bref))
        self.Pos[1]=float(nextpos[1]+self.p*m.sin(self.b+self.bref))
        self.Pos[2]= float(-(self.b+self.bref+self.a))
        return self.Pos[0]

    def check(self):
        if self.p<5 and self.node<(len(self.global_path)-2):
            print(self.p)
            #compute a new alpha,beta,gamma
            self.node=self.node+1
            print('node',self.node)
            self.compute_pba()
        return self.node



    def thymio(self,Ts,th):
        while True:
            self.compute_state_equation(Ts)
            self.compute_Pos()
            #print('POS',self.Pos)

            self.check()
            self.compute_input()
            self.run_on_thymio(th)

            time.sleep(Ts)

            if self.p<1 and self.node==len(self.global_path)-2:
                th.set_var("motor.left.target", 0)
                th.set_var("motor.right.target", 0)
        return self.Pos[0]
