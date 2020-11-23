# -*- coding: utf-8 -*-
"""
Created on Wed Nov 18 14:34:27 2020

@author: t
"""
import cv2
import numpy as np
import skimage
import matplotlib as plt
from skimage import morphology
import Vision as v
from scipy import linalg

input_path = "2.jpg"
output_path = "output.avi"
DISPLAY_GRAPH = True

def get_first_frame(input):
    video = cv2.VideoCapture(input)
    if(video.isOpened() == False):
        print("Error opening video")

    #get first frame for object recognition
    video.set(1, 0)
    ret, frame = video.read()
    return frame
def get_image(input):
    frame = cv2.imread(input)
    small_frame = cv2.resize(frame,(624,416))
    return np.array(small_frame)



frame =get_image(input_path)
#print(frame.shape)
filr = v.colorfilter("RED")
filg = v.colorfilter("GREEN")
filb = v.colorfilter("BLUE")
fily = v.colorfilter("YELLOW")
maskr= filr.get_mask(frame)
BL = v.getCentroid(maskr)
maskg= filg.get_mask(frame)
TL = v.getCentroid(maskg)
maskb= filb.get_mask(frame)
BR = v.getCentroid(maskb)
masky= fily.get_mask(frame)
TR = v.getCentroid(masky)


corners = np.array([TL,TR,BL,BR], np.float32)
trans = v.projection(corners)

if DISPLAY_GRAPH:
    
    #real centroids
    cv2.circle(frame, (int(TL[0]),int(TL[1])), 15, (0,255,0), thickness=3, lineType=8, shift=0)
    cv2.circle(frame, (int(BL[0]),int(BL[1])), 15, (0,255,0), thickness=3, lineType=8, shift=0)
    cv2.circle(frame, (int(TR[0]),int(TR[1])), 15, (0,255,0), thickness=3, lineType=8, shift=0)
    cv2.circle(frame, (int(BR[0]),int(BR[1])), 15, (0,255,0), thickness=3, lineType=8, shift=0)
    
    #calculated centroid
    cTL = v.applyTransform(linalg.inv(trans),np.array([0,0]))
    cTR  = v.applyTransform(linalg.inv(trans),np.array([95,5]))
    cBL  = v.applyTransform(linalg.inv(trans),np.array([5,95]))
    cBR  = v.applyTransform(linalg.inv(trans),np.array([95,95]))
    cv2.circle(frame, tuple(cTL.astype(int)), 15, (255,0,0), thickness=3, lineType=8, shift=0)
    cv2.circle(frame, tuple(cTR.astype(int)), 15, (255,0,0), thickness=3, lineType=8, shift=0)
    cv2.circle(frame, tuple(cBL.astype(int)), 15, (255,0,0), thickness=3, lineType=8, shift=0)
    cv2.circle(frame, tuple(cBR.astype(int)), 15, (255,0,0), thickness=3, lineType=8, shift=0)
    
    
    
    cv2.imshow("test",frame)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
