""" Developped by : Pierre """

import os
import sys
import time
import serial
import numpy as np
import math as m

from Thymio import Thymio
from Timer import RepeatedTimer
from Robot import Robot
from tqdm import tqdm

class Filtering:

    def __init__(self, Rvel, Rcam, Q, robot, Hvel, Ts):
        self.Rvel = Rvel
        self.Rcam = Rcam
        self.Q = Q
        self.Hcam = np.zeros((2,4))
        self.Hcam[0][0] = 1
        self.Hcam[1][1] = 1
        self.Hvel = Hvel
        self.Pest_priori = Q
        self.robot = robot
        self.Ts = Ts
    
    @staticmethod
    def update(X_est,P_est_priori, zk, H, A, R):

        innovation = zk - np.dot(H,X_est)
        #print("S")
        S = np.dot(H, np.dot(P_est_priori, H.T)) + R
        #print(S)
        #print("K")
        K = np.dot(P_est_priori, np.dot(H.T, np.linalg.inv(S)))
        #print(K)
        X_real = X_est + np.dot(K,innovation)
        #print(X_est)
        P_est = P_est_priori - np.dot(K,np.dot(H, P_est_priori))

        return X_real, P_est

    def kalman(self, Xcam, Xest):
        """Xcam is measured state with camera,
           X_est is predicted state from a priori state (by State Space), 
           A, B, are state space parameters, 
           V is the velocity"""

        theta = robot.Pos[1]
        vR_measured = self.robot.th.get_var('motor.right.speed')
        VL_measured = self.robot.th.get_var('motor.left.speed')

        V_measured = np.array([[vR_measured],[vL_measured]])
        
        A = np.array([[1, 0, 0, self.Ts*m.cos(theta)/2, self.Ts*m.cos(theta)/2],[0, 1, 0, self.Ts*m.cos(theta)/2, self.Ts*m.sin(theta)/2],
                     [0, 0, 1, 1/(2*4.7), 1/(2*4.7)],[0, 0, 0, 1, 0],[0, 0, 0, 0, 1]]) 

        
        #print(X_est)
        P_est = np.dot(A,np.dot(self.Pest_priori,A.T)) + self.Q
        #print(P_est)


        #Update for velocity sensor

        X_est, P_est = self.update(X_est, P_est,V_measured, self.Hvel, A, self.Rvel)

        #Update for camera sensor

        X_est, P_est = self.update(X_est, P_est, Xcam, self.Hcam, A, self.Rcam)


        #return
        self.Pest_priori = P_est

        return X_est

    def filter(self,enable,camera_position,state_estimator,target = False):
        """ input:      the position as computed by the vision algorithm (a 3d numpy vector)
                        the position estimated by the state estimator (a 3d numpy vector), 

            returns:    an estimation of position a 3d numpy vector """