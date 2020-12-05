from Robot import Robot
import os
import sys
import time
import numpy as np
import math as m
import pandas as pd
import math as m
import serial
from Utilities import Utilities
# Import the necessary packages and modules
import matplotlib.pyplot as plt

ut = Utilities()


global_path = [(10.,10.),(10.,50.),(30,45),(60,70)]
Startx=global_path[0][0]
Starty=global_path[0][1]
Goalx=global_path[-1][0]
Goaly=global_path[-1][1]

# Initialise robot class

Init_pos = np.array([10.,10.,m.pi/4])
Ts = 0.1
kp = 1.5 #3    #0.15   #0.5
ka = 18 #35  #0.4    #0.8
kb = -4 #-8   #-0.07  #-0.2
vTOm=30.30
wTOm=(200*180)/(80*m.pi) #130.5 #

thym = Robot(global_path,Init_pos,Ts, kp,ka,kb,vTOm,wTOm)

thym.compute_path()

# Plot the data
plt.plot(thym.pathcontrolx, thym.pathcontroly, 'b-',thym.Global_x,thym.Global_y,'ro')
plt.plot(Startx,Starty,'go',markersize=15)
plt.plot(Goalx,Goaly,'rX',markersize=15)

# Add a legend
plt.title('thymio trajectory')
# y axis down : 
plt.gca().invert_yaxis()

# Show the plot : 
plt.show()