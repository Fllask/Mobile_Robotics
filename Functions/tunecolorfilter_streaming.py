# -*- coding: utf-8 -*-
"""
Created on Tue Nov 24 23:43:25 2020

@author: valla
"""

import Vision as v
import cv2
import math
import numpy as np
import time
CAMERA = "ANDROID FLASK"
cap = cv2.VideoCapture(1)

# while(1):
    
    #ret,img = cap.read()
img = cv2.imread('../sample_pictures/new color.jpg')
img = v.preprocess(img)
imgbgr = cv2.cvtColor(img,cv2.COLOR_HSV2BGR)
filterg = v.colorfilter("YELLOW", camera =CAMERA)
#v.manually_get_centroid(img,preprocessed = True)
mask = filterg.get_mask(img)
cv2.imshow('mask auto',mask)
GC, flag = v.getCentroid(mask)
if flag:
    GC = v.manually_get_centroid(img, preprocessed=True)
cv2.namedWindow('filtered',cv2.WINDOW_NORMAL)
cv2.resizeWindow('filtered', 624,416)
cv2.circle(imgbgr, (int(GC[0]),int(GC[1])), 15, (0,255,0), thickness=3, lineType=8, shift=0)
cv2.imshow('filtered', imgbgr)
#     if cv2.waitKey(1) & 0XFF == ord('q'):
#         break 
# cap.release()
while(not(cv2.waitKey(1) & 0XFF == ord('q'))):

    if False:
        print("L")

cv2.destroyAllWindows()
