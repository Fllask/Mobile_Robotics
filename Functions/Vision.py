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
from skimage import color, filters, measure, util, morphology
from scipy import linalg

CIRCLE = 1
SQUARE = 2
POLYGON = 3
BLACK = np.array([0,0,0])
WHITE = np.array([255,255,255])
RED = np.array([255,0,0])


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
        map = Features()
        return map.getObstacles()

    def returnDynamicCoordinates(self):
        """ returns an object containing the coordinates of the robot (a 3d numpy vector) 
        as well as the end point coordinates (a 2d numpy vector) 
        OR False if the image is not exploitable"""
        coord = Features()
        if coord.isUsable():
            return coord
        else:
            return False
class Features:
    def __init__(self, polygons = np.array([[10,20],[20,10],[50,50],[10,10]]), 
                 robot= np.array([10,10,0]), end = np.array([90,90]),
                 usable = True):
        self.obstacles = polygons
        self.robot = robot
        self.end = end
        self.usable = usable
    def getObstacles():
        return self.obstacles
    def getEnd():
        return self.end
    def getRobotPos():
        return self.robot
    def isUsable():
        return self.usable

    
class colorfilter:
    def __init__(self, color):
        if color == "RED":
            self.band = np.array([[6,29],[8,35],[72,97]])
        if color == "YELLOW":
            self.band = np.array([[0,39],[124,166],[139,181]])
        if color == "BLUE":
            self.band = np.array([[37,65],[11,38],[1,28]])
        if color == "GREEN":
             self.band = np.array([[11,41],[58,94],[46,77]])
        if color == "BLACK":
             self.band = np.array([[0,39],[0,166],[0,181]])
        
    def get_mask(self,image):
        mask = np.zeros((image.shape[0], image.shape[1]))
        for x in range(image.shape[0]):
            for y in range(image.shape[1]):
                pix = image[x][y]
                #print((pix<self.band[:,1]).all())
                if (pix>self.band[:,0]).all() & (pix<self.band[:,1]).all():
                    mask[x,y] = 1
                    #print("CAL")
        morphology.binary_opening(mask, out=mask)
        morphology.binary_closing(mask,out=mask)
        return mask
    
    
def projection(corners):
    #corners: coordinates of TL TR BL BR in the image []
    
    #project TL on (5,5), then calculate the least mean square transformation
    
    rcorners = np.array([[5,5],[95,5],[5,95],[95,95]],np.float32)
    trans = cv2.getPerspectiveTransform(corners, rcorners)
    return trans
def applyTransform(trans,x):
    x = x.reshape(2)
    xp = np.append(x,1)
    yp = np.dot(trans,xp)
    y = (yp/yp[2])[0:2]
    return y

    
def getCentroid(imageBin):
    moments = measure.moments(imageBin, order = 1)
    centroid = np.array([moments[0,1]/moments[0,0], moments[1,0]/moments[0,0]])
    return centroid

def getcoord(imageBin, trans):
    #get the real coordinates of a the robot
    pose = np.zeros(3)
    pose[0:1] = trans.transform(getCentroid(imageBin))
    
    

test = Vision()
#tr = projection(test.corners)

#print(tr.transform(np.array([[0.5,0.5]]).T))