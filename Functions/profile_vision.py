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
import time

def test():
    input_path = '../sample_pictures/test_set_2/03.jpg'
    img = v.get_image(input_path)
    imgprep = v.preprocess(img)
    vis = v.Vision(imgprep)
    rob = np.array([500,500,math.pi/2])