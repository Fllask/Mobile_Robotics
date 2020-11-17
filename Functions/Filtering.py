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

    def __init__(self, Rvel, Rcam, Q, robot, Hvel):
        self.Rvel = Rvel
        self.Rcam = Rcam
        self.Q = Q
        self.Hcam = np.zeros((2,4))
        self.Hcam[0][0] = 1
        self.Hcam[1][1] = 1
        self.Hvel = Hvel
        self.Pest_priori = Q
        self.robot = robot
    
    @staticmethod
    def update(X_est_priori,P_est_priori, zk, H, A, R):
        innovation = zk - np.dot(H,X_est_priori)
        #print("S")
        S = np.dot(H, np.dot(P_est_priori, H.T)) + R
        #print(S)
        #print("K")
        K = np.dot(P_est_priori, np.dot(H.T, np.linalg.inv(S)))
        #print(K)
        X_est = X_est_priori + np.dot(K,innovation)
        #print(X_est)
        P_est = P_est_priori - np.dot(K,np.dot(H, P_est_priori))

        return X_est, P_est

    def kalmanRectiligne(self, Xcam, Xest_priori, measuredSpeed):
        """Xcam is measured state with camera,
           X_est is predicted state from a priori state (by State Space), 
           A, B, are state space parameters, 
           V is the velocity"""
        
        A = self.robot.A
        B = self.robot.B

        
        #Prediction
        X_est = np.dot(A, Xest_priori)
        #print(X_est)
        P_est = np.dot(A,np.dot(self.Pest_priori,A.T)) + self.Q
        #print(P_est)


        #Update for velocity sensor

        X_est, P_est = self.update(X_est, P_est, measuredSpeed, self.Hvel, A, self.Rvel)

        #Update for camera sensor

        X_est, P_est = self.update(X_est, P_est, Xcam, self.Hcam, A, self.Rcam)


        #return
        self.Pest_priori = P_est

        return X_est

    def filter(self,enable,camera_position,state_estimator,target = False):
        """ input:      the position as computed by the vision algorithm (a 3d numpy vector)
                        the position estimated by the state estimator (a 3d numpy vector), 

            returns:    an estimation of position a 3d numpy vector """