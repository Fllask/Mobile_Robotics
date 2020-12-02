#!/Library/Frameworks/Python.framework/Versions/3.8/bin/python3.8
#the line above makes it more nicely executable on my mac, PATH may differ on different machines

import cv2
import Vision
import numpy as np

""" correct webcam for whatever webcam you have here """

webcam = False

if webcam:
    cap = cv2.VideoCapture(2)


def f():
    print("button pressed")

def dumm(x):
    pass

def loadImg(isFile = False,input = "../sample_pictures/test_set_2/06.jpg"):
    if isFile:
        img = cv2.imread(input) 
        return img
    ret, img = cap.read()
    return img


cfilter = Vision.colorfilter("BLACK",camera="ANDROID FLASK")
band = cfilter.band # color band for a given object 


Hmin = band[0][0]
Hmax = band[0][1]
Smin = band[1][0]
Smax = band[1][1]
Lmin = band[2][0]
Lmax = band[2][1]

cv2.namedWindow('trkbar')

cv2.createTrackbar('Hmin','trkbar',0,179,dumm)
cv2.createTrackbar('Hmax','trkbar',0,179,dumm)
cv2.createTrackbar('Smin','trkbar',0,255,dumm)
cv2.createTrackbar('Smax','trkbar',0,255,dumm)
cv2.createTrackbar('Lmin','trkbar',0,255,dumm)
cv2.createTrackbar('Lmax','trkbar',0,255,dumm)
# cv2.createButton("Next color",)


for i in range(1):
    while True:
        
        img = loadImg(isFile = not webcam, input = "../sample_pictures/test_set_2/06.jpg")
        p2, p90 = np.percentile(img, (2, 90))
        img =  cv2.UMat(cv2.resize(img,(624,416)))

        msk = cfilter.get_mask(img) # computing mask from band

        cv2.imshow("raw footage", img);

        # create trackbars for color change

        Hmin = cv2.getTrackbarPos('Hmin','trkbar')
        Hmax = cv2.getTrackbarPos('Hmax','trkbar')
        Smin = cv2.getTrackbarPos('Smin','trkbar')
        Smax = cv2.getTrackbarPos('Smax','trkbar')
        Lmin = cv2.getTrackbarPos('Lmin','trkbar')
        Lmax = cv2.getTrackbarPos('Lmax','trkbar')

        # print(Hmin,Hmax,Smin,Smax,Lmin,Lmax)
        band = np.array([[Hmin,Hmax],[Smin,Smax],[Lmin,Lmax]])
        cfilter.band = band


        cv2.imshow("mask", msk);
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break      