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




# Initialise robot class

global_path = [(0.,0.),(50.,0.),(50.,10.1)] # Point of the global path [(xstart,ystart),....,(xgoal,ygoal)]

Startx=global_path[0][0]
Starty=global_path[0][1]
Goalx=global_path[-1][0]
Goaly=global_path[-1][1]


InitAngle=-m.pi/4
kp = 3 
ka = 35 
kb = -8 
Ts=0.1
# Initialise robot class
Init_pos = np.array([global_path[0][0],global_path[0][1],InitAngle])

vTOm=30.30
wTOm=(200*180)/(80*m.pi) #130.5 #

thym = Robot(global_path,Init_pos,Ts, kp,ka,kb,vTOm,wTOm)

thym.compute_path(Ts)

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