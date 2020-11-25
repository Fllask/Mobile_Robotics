# -*- coding: utf-8 -*-
"""
Created on Tue Nov 24 23:43:25 2020

@author: valla
"""

import Vision as v
import cv2
import math
import numpy as np




for i in range(22):
    
    print(i)
    if i < 10:
        path = 'c:\\users\\valla\\documents\\github\\mobile_robotics'\
            '\\sample_pictures\\test_set_2\\0'+str(i)+'.jpg'
    else:
        path = 'c:\\users\\valla\\documents\\github\\mobile_robotics'\
            '\\sample_pictures\\test_set_2\\'+str(i)+'.jpg'
    img = v.get_image(path)
    
    img = v.preprocess(img)
    imgbgr = cv2.cvtColor(img,cv2.COLOR_HSV2BGR)
    filterg = v.colorfilter("BLACK")
    mask = filterg.get_mask(img)
    cv2.imshow('mask',mask)
    cv2.waitKey(0)
    #GC = v.getCentroid(mask)
    '''cv2.namedWindow('filtered',cv2.WINDOW_NORMAL)
    cv2.resizeWindow('filtered', 624,416)
    if ~np.isnan(GC[0]):
        cv2.circle(imgbgr, (int(GC[0]),int(GC[1])), 15, (0,255,0), thickness=3, lineType=8, shift=0)
    cv2.imshow('filtered', imgbgr)
    cv2.waitKey(0)'''
    cv2.destroyAllWindows()