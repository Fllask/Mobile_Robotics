# -*- coding: utf-8 -*-
"""
Created on Fri Nov 20 19:08:11 2020
Example of using the Vision module and class
@author: Flask
"""

import Vision as v
import cv2

DISPLAY = True


input_path = "2.jpg"
img = v.get_image(input_path)

vis = v.Vision(img)

if DISPLAY:
    img_real = cv2.warpPerspective(img, vis.trans, (1000,1000))
    obstacles = vis.getMap(downscale = False)
    cv2.drawContours(img_real, obstacles, -1, (0,255,0), 3)
    for p in obstacles:
        for corner in p:
            cv2.circle(img_real, tuple(corner.reshape(2)), 5, (255,255,0), thickness=1, lineType=8, shift=0)
    cv2.namedWindow('map',cv2.WINDOW_NORMAL)
    cv2.resizeWindow('map', 600,600)
    cv2.imshow('map', img_real)
    cv2.waitKey(0)
    cv2.destroyAllWindows()