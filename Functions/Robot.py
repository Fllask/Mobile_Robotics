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
        self.ML=0       # value to put in the left part of the motor
        self.MR=0       # value to put on the right wheel motor
        self.state=0    # state of the robot 
    
    
    
    def compute_pba(self):
            
        #print(self.node)
        self.b=-ut.compute_angle(self.Pos[0:2],self.global_path[self.node+1])
        #print('pos1',self.global_path[(self.node)])
        #print('pos2',self.global_path[(self.node+1)])
        #print('self global path',self.global_path)
        self.a=-self.Pos[2]-self.b
        #print('POS',self.Pos[0:2])
        self.p=ut.compute_distance(self.Pos[0:2],self.global_path[self.node+1])
        self.bref=-ut.compute_angle(self.global_path[self.node],self.global_path[self.node+1])
        #print('self b',self.b)
        #print('self.a',self.a)
        #print('self.p',self.p)
        return self.b

    # assume alpha(0)is between -pi/2 and pi/2 and stays between those two values
    def compute_state_equation(self,Ts):
    
        self.p=self.p*(1-Ts*self.kp*m.cos(self.a))
        #print('p after',self.p)
        self.a=self.a+Ts*(self.kp*m.sin(self.a)-self.ka*self.a-self.kb*(self.b-self.bref))
        self.b= self.b-Ts*(self.kp*m.sin(self.a))
        #print('alpha',self.a)
        #print('beta',self.b)




    
    def compute_input(self):

        self.u[0]=self.kp*self.p
        self.u[1]=self.ka*self.a+self.kb*(self.b-self.bref)

        #print('u0',self.u[0])
        #print('u1',self.u[1])
        vM=self.u[0]*self.vTOm
        wM=self.u[1]*self.wTOm

        ML=vM+wM
        MR=vM-wM
        #print('ML',self.ML)
        #print('MR',self.MR)
        ML = ML if ML >= 0 else 2 ** 16 + ML
        ML = int(ML)
        MR = MR if MR >= 0 else 2 ** 16 + MR
        MR = int(MR)

        self.ML=ML
        self.MR=MR

        return self.ML

    def run_on_thymio(self,th):
        th.set_var("motor.left.target", self.ML)
        th.set_var("motor.right.target", self.MR)
        return self.ML

    def compute_Pos(self):
        nextpos=self.global_path[self.node+1]
        self.Pos[0]=float(nextpos[0]-self.p*m.cos(self.b))
        self.Pos[1]=float(nextpos[1]+self.p*m.sin(self.b))
        self.Pos[2]= float(-(self.b+self.a))
        return self.Pos[0]

    def check(self):
        if self.p<20 and self.node<(len(self.global_path)-2):
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
