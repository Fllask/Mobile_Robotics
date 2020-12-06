from Functions.Thymio import Thymio
import os
import sys
import time
import numpy as np
import math as m
import pandas as pd
import math as m
import serial
from Functions.Global import Global
from Functions.Utilities import Utilities
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
        self.node=0                        # use to estimate where the thymio is on the global path 
        
        self.go=1
        self.a=None                        # coordinate of the robot for the astolfi controller
        self.b=None
        self.p=None
        self.vTOm=vTOm                     # constant to convert velocity with value to give to the left and right motor
        self.wTOm=wTOm                     # constant to convert angular velocity with value to give to the left and right motor
        if not isinstance(global_path,bool):
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
        self.sensor = None                 # values for the sensor
    


    def compute_pba(self,verbose = False):
        ''' compute the parameter for the astolfi controller : 
        self.Pos[2] is the angle theta of the robot 
        and bref is the angle we want for the robot at the objectif '''
        
        self.a=-self.Pos[2]+ut.compute_angle(self.Pos,self.global_path[self.node+1])
        self.a=(m.pi+self.a)%(2*m.pi)-m.pi # make sure that we get a value between -pi and pi
        self.p=ut.compute_distance(self.Pos,self.global_path[self.node+1])
        self.bref=-ut.compute_angle(self.global_path[self.node],self.global_path[self.node+1])
        self.b=-self.Pos[2]-self.bref-self.a
        self.b=(m.pi+self.b)%(2*m.pi)-m.pi
        if verbose:
            print("a : " + str(self.a) + " , b : " + str(self.b) + " , p : " + str(self.p)) 
        return self.b

    def astofli_controller(self,p,a,b):
        ''' compute the derivative of rho beta and alpha'''
        p_dot=-self.kp*m.cos(a)
        a_dot=(self.kp*m.sin(a)-self.ka*a-self.kb*b)/p
        b_dot=(-self.kp*m.sin(a))/p
        return p_dot,a_dot,b_dot

    def compute_state_equation(self,Ts):
        ''' use runge Kutta 2 to get rho alpha and beta at time t+Ts '''
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

    def compute_rotation(self,Ts):
        ''' If alpha is not between -pi/2 and pi/2 we make the robot turn on himself before using the astolfi controller '''
        if self.a>0:
            w=0.3
            self.Pos[2]=self.Pos[2]+w*Ts
            self.a=self.a-w*Ts
        else:
            w=-0.3
            self.Pos[2]=self.Pos[2]+w*Ts
            self.a=self.a-w*Ts

        self.Pos[2]=(m.pi+self.Pos[2])%(2*m.pi)-m.pi
        self.u[0]=0
        self.u[1]=w
        if abs(self.a)< 0.1:
            self.state='ASTOLFI'
        
        return self.a
    def checknotoutofboud(self,ML,MR):
        if ML>500:
            ML=500
        if ML<-500:
            ML=-500
        if MR>500:
            MR=500
        if MR<-500:
            MR=-500
        return ML,MR
        
    def compute_input(self):
        '''function convert the speed and angular speed to the values we need to give the robot'''

        # u[0] is the speed of the robot in cm/s
        # u[1] is the angular speed of the robot in cm/s

        vM=self.u[0]*self.vTOm
        wM=self.u[1]*self.wTOm
        
        MLtest=vM+wM
        MRtest=vM-wM

        [ML,MR]=self.checknotoutofboud(MLtest,MRtest)
        
        ML = ML if ML >= 0 else 2 ** 16 - 1 + ML
        ML = int(ML)
        MR = MR if MR >= 0 else 2 ** 16 - 1 + MR
        MR = int(MR)

        self.ML=ML
        self.MR=MR

    def run_on_thymio(self,th):
        ''' give values to the motors of the thymio'''
        th.set_var("motor.left.target", self.ML)
        th.set_var("motor.right.target", self.MR+2) # to compensate for error between the two wheels
        
        return self.ML
   
    def compute_Pos(self):
        ''' knowing rho alpha and beta we recompute the values for x y and theta that will be send later to the filter '''
        nextpos=self.global_path[self.node+1]
        self.Pos[0]=float(nextpos[0]-self.p*m.cos(self.b+self.bref))
        self.Pos[1]=float(nextpos[1]+self.p*m.sin(self.b+self.bref))
        self.Pos[2]= float(-(self.b+self.bref+self.a))
        self.Pos[2]=(m.pi+self.Pos[2])%(2*m.pi)-m.pi
        return self.Pos[0]

    def check(self):
        ''' check if we have arrived at our objective and set up our next objective (next point in the global path)'''
        if self.p<5 and self.node<(len(self.global_path)-2):
            self.node=self.node+1   #go to the next point in the global path
            
            self.compute_pba()      #compute a new alpha,beta,gamma
        return self.node

    # FUNCTIONs USED FOR THE LOCAL AVOIDANCE : 

    def compute_straight_local(self,Ts,v):
        ''' compute the new postition of the robot when it goes straight '''
        self.Pos[0]=self.Pos[0]+m.cos(self.Pos[2])*v*Ts
        self.Pos[1]=self.Pos[1]+m.sin(self.Pos[2])*v*Ts
        return self.u[0]
    #function used to turn left or right
    def compute_rot_local(self,Ts,w):
        ''' compute the new angle theta of the robot when it turn on itlself'''
        self.Pos[2]=self.Pos[2]+w*Ts
        self.Pos[2]=(m.pi+self.Pos[2])%(2*m.pi)-m.pi
        return self.Pos[2]
   
    def check_localobstacle(self,th) :
        ''' function use to check if we detect a local obstacle :
        if the values of the sensor are above a treshold the robot goes in local mode'''

        self.sensor= np.array(th["prox.horizontal"]) #get values from the sensors
        if self.sensor[0]>1000 or self.sensor[1]>1000 or self.sensor[2]>3000 or self.sensor[3]>1000 or self.sensor[4]>1000: # threshold a modifi� 
            self.state='LOCAL'
            right=self.sensor[4]+self.sensor[3]   #values of the right self.sensors   
            left=self.sensor[0]+self.sensor[1]    #values of the left self.sensors
            if right>left:              #turn left if it feels the object on the right
                self.turn=0             #turn =0 means turn left
                self.idx_sensor=(3,4)   # index of the sensor that we will use : 3 and 4
            else:                       #turn right if it feels the object on the left
                self.turn=1
                self.idx_sensor=(1,0)   
        return self.state
  
    def checkstate0(self,th):
        ''' check if we are still in localstate 0 (turn left or right) or if we can go to localstate 1 (go straight) '''
        self.sensor= np.array(th["prox.horizontal"])
        if self.sensor[self.idx_sensor[1]]<1 and self.sensor[self.idx_sensor[0]]<1:
            self.locstate=1
            self.cnt=1
        return self.locstate
 
    def checkstate2(self,th):
        '''check if we are still in localstate 2 (turn right or left) or if we can go to localstate 0 (turn left or right)
        turn in the direction of the obstacle until we feel it then go back in local state 0 to turn in the other side
        this allow us to stay close to get around the obstacle'''

        self.sensor= np.array(th["prox.horizontal"])
        if self.sensor[self.idx_sensor[1]]>700:
            self.locstate=0
        return self.locstate

    def checkout(self,th):
        ''' check if we still need to be in local avoidance or if we can go in global avoidance 
        if the thymio is pointing to the next goal and that we don't feel any local obstacle we go back in global avoidance'''

        angle=ut.compute_angle(self.Pos[0:2],self.global_path[self.node+1])

        test=(m.pi+self.Pos[2]-angle)%(2*m.pi)-m.pi
        if abs(test)<0.1:
            self.state='INIT'
            self.u[0]=0
            self.u[1]=0

            th.set_var('motor.left.target',0)
            th.set_var('motor.right.target',0)
            self.locstate=0
                

        return self.state

    def compute_path(self,Ts):
        ''' Function use to simulate the path our robot will take using the astolfi controller'''
        go=1
        self.pathcontrolx=[self.Pos[0]]
        self.pathcontroly=[self.Pos[1]]
        self.Global_x=[self.global_path[0][0]]
        self.Global_y=[self.global_path[0][1]]

        while go:


            # compute rho, alpha and beta at time t+ts
            self.compute_state_equation(Ts)
            # convert rho, beta and alpha in x y and theta (need those parameters for the filter)
            self.compute_Pos()

            if abs(self.a)>m.pi/2:   
                self.a=0
                self.Pos[2]=ut.compute_angle(self.Pos,self.global_path[self.node+1])
            # check if we are close to the next point in the global path and change the next goal in the astolfi controller if it is the case
            self.check()
            self.pathcontrolx.append(self.Pos[0])
            self.pathcontroly.append(self.Pos[1])
            if self.p<3 and self.node==len(self.global_path)-2:
                go=0

        for i in range(1,len(self.global_path)):
            self.Global_x.append(self.global_path[i][0])
            self.Global_y.append(self.global_path[i][1])

       

    def INIT(self,global_path, pos_init) : 
        ''' the robot stays in INIT state until it gets a global path and an initial position
        global path : give false if no global path else it gives the global path in a tuple
        pos_init: the initial position (x,y,theta)'''

        self.state = 'INIT'
        if global_path is not False :
            self.global_path = global_path
        else :
            return self.state
        if pos_init is not False :
            self.Pos = pos_init
        else : 
            return self.state
        self.node = 0
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

    def ASTOLFI(self,th,Ts,filter,pos_cam, update_cam):
        ''' Astolfi controller with constant speed is used to control the robot when the angle alpha is between -pi/2 and pi/2
             th : serial link to the thymio
             Ts : time of one iteration of the loop while (we recompute every Ts)
             filter: a kalman filter is used using the measurement of the left and right motor and the measurement of the camera
             pos_cam: measurement of the camera'''
        
        #[give : x,y,theta,vr,vl] to the filter : 
        vect = self.get_states()

        # check if we detect a local obstacle
        self.check_localobstacle(th)
        
        if self.state == 'LOCAL' :
            return self.state

        # converting x,y and theta in rho, beta and alpha (Astolfi controller)
        self.compute_pba()

        if abs(self.a)>m.pi/2:
            # calculate rho, beta and alpha at time t+1(Astolfi controller)
            self.state='TURN'
            return self.state


        #check for kidnapping
        elif pos_cam is not False and pos_cam[0] != 0 and np.linalg.norm(pos_cam[0:2] - vect[0:2],2) > 10.:
            self.state = 'INIT'
            th.set_var('motor.left.target',0)
            th.set_var('motor.right.target',0)
            time.sleep(3)
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

            # get our pos with the filter
            filter.compute_kalman(pos_cam,vect,th,Ts,update_cam)
            return self.Pos
        
    def TURN(self,th,Ts,filter, pos_cam, update_cam):

        '''if abs(alpha)>pi/2 we can't use astolfi and we first need to rotate the robot on itslef. We make it turn on  itself until alpha 
            is close to 0
           th : serial link to the thymio
           Ts : time of one iteration of the loop while (we recompute every Ts)'''
        

        #check for kidnapping
        vect = self.get_states()

        if pos_cam is not False and pos_cam[0] != 0 and np.linalg.norm(pos_cam[0:2] - vect[0:2],2) > 10.:
            self.state = 'INIT'
            th.set_var('motor.left.target',0)
            th.set_var('motor.right.target',0)
            time.sleep(3)
            return self.state

        self.compute_rotation(Ts)
        # calculate the velocity and angular velocity and the value we need to give to the left and right motor
        self.compute_input()

        # give the value of the motor to the thymio 
        self.run_on_thymio(th)

        # sleep 0.1 second :
        time.sleep(0.1)

        #[give : x,y,theta,vr,vl] to the filter : 
        vect = self.get_states()

        # sleep 0.1 second :
        time.sleep(0.1)

        # get our pos with the filter
        filter.compute_kalman(pos_cam,vect,th,Ts,update_cam)

        return self.state

    def LOCAL(self,th,Ts, filter, pos_cam, update_cam):
        ''' we get around the local obstacle  until we we don't detect any obstacle in front of the robot when the robot is oriented 
        in the direction of the goal
        the robot will enter in init mode until we recompute a global path.
        TO get around the obstacle the robot will use 3 movements: go straight turn left and turn right. it turn first to avoid
        the obstace then goes straight during 2 second then turn in the other size to see if we are still close to the obstacle. 

        th : serial link to the thymio
        Ts : time of one iteration of the loop while (we recompute every Ts)
        '''

        #check for kidnapping
        vect = self.get_states()

        if pos_cam is not False and pos_cam[0] != 0 and np.linalg.norm(pos_cam[0:2] - vect[0:2],2) > 10.:
            self.state = 'INIT'
            th.set_var('motor.left.target',0)
            th.set_var('motor.right.target',0)
            time.sleep(3)
            return self.state

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
            self.u[0]=2 # go straight
            self.u[1]=0
            # go straight
            self.compute_straight_local(Ts,self.u[0])
            if self.cnt==20:
                self.locstate=2
            self.checkout(th)
           
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

        if self.locstate==0:
            #[give : x,y,theta,vr,vl] to the filter : 
            vect = self.get_states()

            # get our pos with the filter
            filter.compute_kalman(pos_cam,vect,th,Ts,update_cam)

        return self.state

    def FINISH(self,th, go) :
        if self.p is not None :
            if self.p<5 and self.node==len(self.global_path)-2:
                th.set_var("motor.left.target", 0)
                th.set_var("motor.right.target", 0)
                print('FININSH!!!!')
                go = 0
        return go