# -*- coding: utf-8 -*-
"""
Created on Tue Nov 24 18:01:12 2020

@author: valla
"""


import Vision as v
import cv2
import math
import numpy as np


input_path = "../sample_pictures/test_set_2/03.jpg"
img = v.get_image(input_path)
imgbw = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
ret, imgb = cv2.threshold(imgbw,128,1,0)
rob = v.getRobotPos(imgb)


pt1 = (int(rob[0]), int(rob[1]))
pt2 = (int(rob[0]+math.cos(rob[2])*100), int(rob[1]+math.sin(rob[2])*100))
cv2.line(img,pt1,pt2,(128,128,0),thickness=3)
cv2.circle(img,pt1,10,(128,128,0),thickness = 4)
cv2.namedWindow('map',cv2.WINDOW_NORMAL)
cv2.resizeWindow('map', 600,600)
cv2.imshow('map', img)
cv2.waitKey(0)
cv2.destroyAllWindows()