""" Developped by: Flask """
import sys
import os  # methods to work with file/directory paths
import getopt #get arguments
import cv2 #read video and images
import numpy as np
import time #wait, sleep
import skimage
import skimage.io as io
import math
import matplotlib.pyplot as plt
from skimage import color, filters, measure, util
from scipy import linalg

class Vision:
    """ Handles vision """
    i = 12345

    def __init__(self):
        self.data = []
        self.corners = np.array([[0,0],[1,0],[0,1],[1,1]]).T
    def f(self):
        corners = 0
        return corners

    def returnMap(self,oldmap=[]):
        """ returns an object containing the map: an array of scipy 
        polygons and the start and end points (2d numpy vectors) """
        return ""

    def returnDynamicCoordinates(self):
        """ returns an object containing the coordinates of the robot (a 3d numpy vector) 
        as well as the end point coordinates (a 2d numpy vector) 
        OR False if the image is not exploitable"""
        return ""
class affine:
    def __init__(self, A =np.array([]), v= np.array([])):
        if A.size == 0:
            self.A = np.array([[1,0],[0,1]])
        else:
            self.A = A
        if  v.size==0:
            self.v = np.array([[100,0]]).T
        else:
            self.v = v
            
    def transform(self,x):
        #return Ax+b
        return np.add(np.dot(self.A,x),self.v)


def projection(corners):
    #corners: coordinates of TL TR BL BR in the image
    
    #project TL on (0,0), then calculate the least mean square transformation
    
    vbp = np.array([corners.T[0,:]]) #translation vector in the image base
    A = np.subtract(corners.T[1:,:],vbp)
    B = np.array([[100,0],[0,100],[100,100]])
    T = linalg.lstsq(A,B)[0]
    v = np.dot(T,vbp.T)
    trans = affine(A = T, v = v)
    return trans

test = Vision()
tr = projection(test.corners)

print(tr.transform(np.array([[0.5,0.5]]).T))