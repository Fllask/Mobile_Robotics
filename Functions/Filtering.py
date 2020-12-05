""" Developped by : Pierre """

import os
import sys
import time
import serial
import numpy as np
import math as m

from Functions.Thymio import Thymio
from Functions.Timer import RepeatedTimer
from Functions.Robot import Robot


class Filtering:

    def __init__(self, Rvel, Rcam, robot, Hvel,Hcam,Ts):
        self.Rvel = Rvel
        self.Rcam = Rcam
        self.robot = robot
        self.Q = np.zeros((5,5))
        # self.compute_Q(Ts,2.5) # Q to measure motor
          
        self.Hcam=Hcam
        self.Hvel = Hvel
        self.Pest_priori = self.Q
        
        
    
    @staticmethod
    def update(X_est,P_est_priori, zk, H, A, R,update_cam, verbose=False):
        
        innovation = zk - np.dot(H,X_est)
       
        if update_cam:
            
            innovation[2]= (m.pi+innovation[2])%(2*m.pi)-m.pi
            print('inn\n', innovation)
        S = np.dot(H, np.dot(P_est_priori, H.T)) + R
        K = np.dot(P_est_priori, np.dot(H.T, np.linalg.inv(S)))
        #print('K\n',K)
        X_est = X_est + np.dot(K,innovation)
        P_est = P_est_priori - np.dot(K,np.dot(H, P_est_priori))
        return X_est, P_est

    def kalman(self, Xcam, X_est,th,Ts,update_cam):
        """Xcam is measured state with camera,
           X_est is predicted state from a priori state (by State Space), 
           A, B, are state space parameters, 
           V is the velocity"""

        theta = self.robot.Pos[1]
        vR_measured = th.get_var('motor.right.speed')
        vL_measured = th.get_var('motor.left.speed')

        if vR_measured>500 or vR_measured<-500:
            vR_measured=0
        if vL_measured>500 or vL_measured<-500:
            vL_measured=0
        
        V_measured = np.array([[vL_measured],[vR_measured]])

        A = np.array([[1, 0, 0, Ts*m.cos(theta)/(2*self.robot.vTOm), Ts*m.cos(theta)/(2*self.robot.vTOm)],
                      [0, 1, 0, Ts*m.sin(theta)/(2*self.robot.vTOm), Ts*m.sin(theta)/(2*self.robot.vTOm)],
                      [0, 0, 1, Ts*1/(2*4.7*self.robot.vTOm), -1*Ts/(2*4.7*self.robot.vTOm)],
                      [0, 0, 0, 1., 0],
                      [0, 0, 0, 0, 1.]]) 

        P_est = np.dot(A,np.dot(self.Pest_priori,A.T)) + self.Q

        #Update for velocity sensor

        X_est, P_est = self.update(X_est, P_est,V_measured, self.Hvel, A, self.Rvel,False)

        #Update for camera sensor
        if update_cam :

            X_est, P_est = self.update(X_est, P_est, Xcam, self.Hcam, A, self.Rcam, update_cam)


        #return
        self.Pest_priori = P_est

        return X_est
    def compute_kalman(self,pos_cam,states_robot,th,Ts,update_cam):
        ''' Function to call for the filtering
            pos_cam : position measured by camera
            states_robot : states of the robot
            th : serial link of the robot to gt vr and vl measured
            Ts : sampling time
            update_cam : boolean to know if we update kalman with the measurements of the camera or not '''

        X_filter=self.kalman(pos_cam,states_robot,th,Ts,update_cam)
        self.compute_Q(Ts, cov_type = {'iso' : 0.5,
                                       'sig1': 0.5,
                                       'sig2': 0.5,
                                       'sig3': 0.02,
                                       'sig4': 0.6,
                                       'sig5': 0.6
                                       })
        self.robot.Pos=X_filter[0:3]
        self.robot.ML=X_filter[3]
        self.robot.MR=X_filter[4]

    def compute_Q(self,Ts, cov_type = {'diag' : 2.}):

       theta = self.robot.Pos[2]
       vTOm = self.robot.vTOm

       #if 'full' in cov_type :

           
       if 'diag' in cov_type :
            self.Q = np.eye(5,5) * cov_type['diag']

       elif 'iso' in cov_type :

            self.Q = np.eye(5,5)
            self.Q[0][0] = cov_type['sig1']
            self.Q[1][1] = cov_type['sig2']
            self.Q[2][2] = cov_type['sig3']
            self.Q[3][3] = cov_type['sig4']*vTOm
            self.Q[4][4] = cov_type['sig5']*vTOm

       elif 'matlab' in cov_type :
            sig = cov_type['matlab']

            self.Q[0][0] = sig*(m.cos(theta)**2)*(Ts**3)/(3*2*(vTOm**2))
            self.Q[0][1] = sig*(m.cos(theta)*m.sin(theta))*(Ts**3)/(3*2*(vTOm**2))
            self.Q[0][2] = 0
            self.Q[0][3] = sig*m.cos(theta)*(Ts**2)/(2*2*(vTOm))
            self.Q[0][4] = sig*m.cos(theta)*(Ts**2)/(2*2*(vTOm))

            self.Q[1][0] = self.Q[0][1] 
            self.Q[1][1] = sig*(m.sin(theta)**2)*(Ts**3)/(3*2*(vTOm**2))
            self.Q[1][2] = 0
            self.Q[1][3] = sig*m.sin(theta)*(Ts**2)/(2*2*(vTOm))
            self.Q[1][4] = sig*m.sin(theta)*(Ts**2)/(2*2*(vTOm))
    
            self.Q[2][0] = self.Q[0][2]
            self.Q[2][1] = self.Q[1][2]
            self.Q[2][2] = sig*(Ts**3)/(3*2*4.7**2*(vTOm**2))
            self.Q[2][3] = sig*Ts**2/(2*2*4.7*(vTOm))
            self.Q[2][4] = - sig*Ts**2/(2*2*4.7*(vTOm))
    
            self.Q[3][0] = self.Q[0][3]
            self.Q[3][1] = self.Q[1][3]
            self.Q[3][2] = self.Q[2][3]
            self.Q[3][3] = sig*Ts
            self.Q[3][4] = 0
        

            self.Q[4][0] = self.Q[0][4]
            self.Q[4][1] = self.Q[1][4]
            self.Q[4][2] = self.Q[2][4]
            self.Q[4][3] = self.Q[3][4]
            self.Q[4][4] = sig*Ts

            print(self.Q)
       return self.Q

