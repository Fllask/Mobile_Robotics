# -*- coding: utf-8 -*-
"""
Created on Wed Nov 18 14:34:27 2020
Different visualisation of the image processing part of the project
@author: t
"""
import cv2
import numpy as np
import skimage
import matplotlib as plt
from skimage import morphology
import Vision as v
from scipy import linalg

input_path = 'c:\\users\\valla\\documents\\github\\mobile_robotics'\
            '\\sample_pictures\\test_set_2\\08.jpg'
DISPLAY_GRAPH = True
RECOMPUTE_PROJ = True


def get_image(input):
    frame = cv2.imread(input)
    
    img = v.preprocess(frame)

    return np.array(img)



frame =  get_image(input_path)
filpoly = v.colorfilter("BLACK")
maskpoly = filpoly.get_mask(frame)
if RECOMPUTE_PROJ:
    filr = v.colorfilter("RED")
    filg = v.colorfilter("GREEN")
    filb = v.colorfilter("BLUE")
    fily = v.colorfilter("YELLOW")
    maskr= filr.get_mask(frame)
    r = v.getCentroid(maskr)
    BL,f = v.getCentroid(maskr)
    maskg= filg.get_mask(frame)
    TL,f = v.getCentroid(maskg)
    maskb= filb.get_mask(frame)
    BR,f = v.getCentroid(maskb)
    masky= fily.get_mask(frame)
    TR,f = v.getCentroid(masky)
    

    corners = np.array([TL,TR,BL,BR], np.float32)
    trans = v.projection(corners)
else:
    trans = np.array([[ 7.52865873e-01,  3.45472605e-02, -1.63045205e+02],
                      [ 4.71914207e-02,  5.56908314e-01, -5.23692719e+01],
                      [ 2.26849070e-03, -1.34130654e-04,  1.00000000e+00]])

polyproj	=	cv2.warpPerspective(maskpoly, trans, (1000,1000))
R_ROBOT = 65
ret, polyprojbin = cv2.threshold(polyproj,2,1,0)
margin = morphology.binary_dilation(polyproj,selem = morphology.disk(R_ROBOT))

polyprojbin = margin.astype(np.uint8)
polycont, ret = cv2.findContours(polyprojbin, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
polynomes = []
for p in polycont:
    polynome    =  cv2.approxPolyDP(p, 15, True)
    polynomes.append(polynome)
#print(polypoly)

if DISPLAY_GRAPH:
    frame = cv2.cvtColor(frame,cv2.COLOR_HSV2BGR)
    frameproj = cv2.warpPerspective(frame, trans, (1000,1000))
    
    cv2.drawContours(frameproj, polynomes, -1, (0,255,0), 3)
    for p in polynomes:
        for corner in p:
            cv2.circle(frameproj, tuple(corner.reshape(2)), 5, (255,255,0), thickness=1, lineType=8, shift=0)
    if RECOMPUTE_PROJ:
    #real centroids
        cv2.circle(frame, (int(TL[0]),int(TL[1])), 15, (0,255,0), thickness=3, lineType=8, shift=0)
        cv2.circle(frame, (int(BL[0]),int(BL[1])), 15, (0,255,0), thickness=3, lineType=8, shift=0)
        cv2.circle(frame, (int(TR[0]),int(TR[1])), 15, (0,255,0), thickness=3, lineType=8, shift=0)
        cv2.circle(frame, (int(BR[0]),int(BR[1])), 15, (0,255,0), thickness=3, lineType=8, shift=0)
        
    #calculated centroid
    cTL = v.applyTransform(linalg.inv(trans),np.array([50,50]))
    cTR  = v.applyTransform(linalg.inv(trans),np.array([950,50]))
    cBL  = v.applyTransform(linalg.inv(trans),np.array([50,950]))
    cBR  = v.applyTransform(linalg.inv(trans),np.array([950,950]))
    cv2.circle(frame, tuple(cTL.astype(int)), 15, (255,0,0), thickness=3, lineType=8, shift=0)
    cv2.circle(frame, tuple(cTR.astype(int)), 15, (255,0,0), thickness=3, lineType=8, shift=0)
    cv2.circle(frame, tuple(cBL.astype(int)), 15, (255,0,0), thickness=3, lineType=8, shift=0)
    cv2.circle(frame, tuple(cBR.astype(int)), 15, (255,0,0), thickness=3, lineType=8, shift=0)
    
    cv2.namedWindow('image',cv2.WINDOW_NORMAL)
    cv2.resizeWindow('image', 600,600)
    cv2.imshow('image', margin.astype(float))
    cv2.namedWindow('frame',cv2.WINDOW_NORMAL)
    cv2.resizeWindow('frame', 600,600)
    cv2.imshow('frame', frameproj)
    
    cv2.imshow("test",frame)
cv2.waitKey(0)
cv2.destroyAllWindows()
