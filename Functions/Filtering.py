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
        self.Rgps = Rcam
        self.Q = Q
        self.Hcam = np.eye(2)
        self.Hvel = Hvel
        self.Pest_priori = Q
        self.robot = robot
    
    @staticmethod
    def update(X_est_priori,P_est_priori, zk, H, A):
        """Update cov and mean matrices for the velocity measurements"""
        innovation = zk - np.dot(H,X_est_priori)
        #print("S")
        S = np.dot(A, np.dot(P_est_priori, A.T))
        #print(S)
        #print("K")
        K = np.dot(P_est_priori, np.dot(H.T, np.linalg.inv(S)))
        #print(K)
        X_est = X_est_priori + np.dot(K,innovation)
        #print(X_est)
        P_est = P_est_priori - np.dot(K,np.dot(H, P_est_priori))

        return X_est, P_est

    def kalmanRectiligne(self, Xcam, Xest_priori, appliedSpeed, measuredSpeed):
        """Xcam is measured state with camera,
           X_est is predicted state from a priori state (by State Space), 
           A, B, are state space parameters, 
           V is the velocity"""
        
        A = self.robot.A
        B = self.robot.B

        
        #Prediction
        X_est = np.dot(A, Xest_priori) + np.dot(B,appliedSpeed)
        #print(X_est)
        P_est = np.dot(A,np.dot(self.Pest_priori,A.T)) + self.Q
        #print(P_est)


        #Update for velocity sensor

        X_est, P_est = self.update(X_est, P_est, measuredSpeed, self.Hvel, A)

        #Update for camera sensor

        X_est, P_est = self.update(X_est, P_est, Xcam, self.Hcam, A)


        #return
        self.Pest_priori = P_est

        return X_est

    def filter(self,enable,camera_position,state_estimator,target = False):
        """ input:      the position as computed by the vision algorithm (a 3d numpy vector)
                        the position estimated by the state estimator (a 3d numpy vector), 

            returns:    an estimation of position a 3d numpy vector """