# -*- coding: utf-8 -*-
"""
Created on Tue Nov 24 23:43:25 2020

@author: valla
"""

import Vision as v
import cv2
import math
import numpy as np

input_path = 'c:\\users\\valla\\documents\\github\\mobile_robotics\\sample_pictures\\'\
             'test_set_2\\19.jpg'
img = v.get_image(input_path)
img = v.preprocess(img)

img = cv2.cvtColor(img,cv2.COLOR_HSV2BGR)
cv2.namedWindow('filtered',cv2.WINDOW_NORMAL)
cv2.resizeWindow('filtered', 624,416)
cv2.imshow('filtered', img)
cv2.waitKey(0)
cv2.destroyAllWindows()