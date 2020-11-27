#!/Library/Frameworks/Python.framework/Versions/3.8/bin/python3.8
# -*- coding: utf-8 -*-
"""
Created on Fri Nov 20 19:08:11 2020
Example of using the Vision module and class
@author: Flask
"""

import Vision as v
import cv2
import math
import numpy as np
DISPLAY = True


input_path = '../sample_pictures/test_set_2/03.jpg'
img = v.get_image(input_path)
imgprep = v.preprocess(img)
vis = v.Vision(imgprep)
rob = np.array([500,500,math.pi/2])
if DISPLAY:
    img_real = cv2.warpPerspective(img, vis.trans, (1000,1000))
    obstacles = vis.getMap(downscale = False)
    cv2.drawContours(img_real, obstacles, -1, (0,255,0), 3)
    for p in obstacles:
        for corner in p:
            cv2.circle(img_real, tuple(corner.reshape(2)), 5, (255,255,0), thickness=1, lineType=8, shift=0)
    pt1 = (int(rob[0]), int(rob[1]))
    pt2 = (int(rob[0]+math.cos(rob[2])*100), int(rob[1]+math.sin(rob[2])*100))
    cv2.line(img_real,pt1,pt2,(128,128,0),thickness=3)
    cv2.circle(img_real,pt1,10,(128,128,0),thickness = 4)
    cv2.namedWindow('map',cv2.WINDOW_NORMAL)
    cv2.resizeWindow('map', 600,600)
    cv2.imshow('map', img_real)
    cv2.waitKey(0)
    cv2.destroyAllWindows()