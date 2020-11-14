""" Developped by : Pierre """

import os
import sys
import time
import serial
import numpy as np

from Thymio import Thymio
from Timer import RepeatedTimer
from tqdm import tqdm

class Filtering:

    def __init__(RspeedVar, LspeedVar, thetaVar):
        # Stock the variances obtain with precedent 
        self.RspeedVar = RspeedVar
        self.LspeedVar = LspeedVar
        self.thetaVar = thetaVar


    @staticmethod
    def getData():
        thymio_data.append({"ground":th["prox.ground.reflected"], 
                            "sensor":th["prox.ground.reflected"],
                            "left_speed":th["motor.left.speed"],
                            "right_speed":th["motor.right.speed"]})
    

    def filter(self,enable,camera_position,state_estimator,target = False):
        """ input:      the position as computed by the vision algorithm (a 3d numpy vector)
                        the position estimated by the state estimator (a 3d numpy vector), 

            returns:    an estimation of position a 3d numpy vector """