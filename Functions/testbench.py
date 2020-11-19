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
input_path = "1.jpg"
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


corners = np.array([TL,TR,BL,BR]).T
trans = v.projection(corners)

point = np.array([[50,50]]).T
realpoint = trans.transform(TL).astype(int).reshape(2)
print(tuple(realpoint.reshape((2))))
if DISPLAY_GRAPH:
    cv2.circle(frame, (int(TL[1]),int(TL[0])), 15, (0,255,0), thickness=3, lineType=8, shift=0)
    cv2.circle(frame, (int(BL[1]),int(BL[0])), 15, (0,255,0), thickness=3, lineType=8, shift=0)
    cv2.circle(frame, (int(TR[1]),int(TR[0])), 15, (0,255,0), thickness=3, lineType=8, shift=0)
    cv2.circle(frame, (int(BR[1]),int(BR[0])), 15, (0,255,0), thickness=3, lineType=8, shift=0)
    cv2.circle(frame, (realpoint[1], realpoint[0]), 15, (255,255,0), thickness=3, lineType=8, shift=0)
    
    cv2.imshow("test",frame)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
