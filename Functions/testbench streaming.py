# -*- coding: utf-8 -*-
"""
Created on Sun Nov 29 12:46:17 2020

@author: valla
"""

import numpy as np
import cv2
import Vision as v
import math
import time
DISPLAY = True


cap = cv2.VideoCapture(2)

#frame = cv2.imread('../sample_pictures/new color.jpg')
initfailed = True
while(initfailed):
    ret,frame = cap.read()
    vis = v.Vision(frame, "ANDROID FLASK")
    initfailed = vis.invalid
    
    if DISPLAY:
        cv2.imshow("frame",frame)
    if cv2.waitKey(1) & 0XFF == ord('q'):
        cv2.destroyAllWindows()
        exit(1)
cv2.destroyAllWindows()
print("INIT OK")
while (True):
    t0 = time.process_time()
    ret,frame = cap.read()
    img_cul = cv2.resize(frame,(624,416))
    vis = v.Vision(frame, "ANDROID FLASK",prevtrans = vis.trans)
    vis.setframe(frame)
    rob,coordvalid = vis.returnDynamicCoordinates()

    if DISPLAY:
        cv2.imshow('frame',img_cul)
        img_real = cv2.warpPerspective(img_cul, vis.trans, (500,500))
        obstacles = vis.getMap(downscale = False)
        if not isinstance(obstacles,bool):
            cv2.drawContours(img_real, obstacles, -1, (0,255,0), 3)
            for p in obstacles:
                for corner in p:
                    cv2.circle(img_real, tuple(corner.reshape(2)), 5, (255,255,0), thickness=1, lineType=8, shift=0)
        if coordvalid:
            pt1 = (int(rob[0]*5), int(rob[1]*5))
            pt2 = (int(rob[0]*5+math.cos(rob[2])*50), int(rob[1]*5+math.sin(rob[2])*50))

            cv2.line(img_real,pt1,pt2,(128,128,0),thickness=3)
            cv2.circle(img_real,pt1,10,(128,128,0),thickness = 4)


        cv2.imshow('map', img_real)
        cv2.namedWindow('internal map',cv2.WINDOW_NORMAL)
        cv2.resizeWindow('internal map', 600,600)
        cv2.imshow('internal map', cv2.cvtColor(vis.frame,cv2.COLOR_HSV2BGR))
    t1 = time.process_time()
    #print(t1-t0)
    if cv2.waitKey(1) & 0XFF == ord('q'):
        break 

cap.release()
cv2.destroyAllWindows()
