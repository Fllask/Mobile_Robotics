import Functions.Robot
import Functions.Filtering
import numpy as np
import math as m

# Initialise robot class

global_path = [(0,0),(60,31.5)]
Init_pos = np.array([0.,0.,0.])
Ts = 0.1
kp = 0.15
ka = 0.4
kb = -0.07

thym = Robot(global_path,Init_pos,Ts, kp,ka,kb)

# Initialise Filtering class
Q = np.array([[1.,0.,0.,0.,0.],[0.,1.,0.,0.,0.],[0.,0.,1.,0.,0.],[0.,0.,0.,6.15,0.],[0.,0.,0.,0.,6.15]])
Rvel = np.array([[6.15, 0.], [0.,6.15]])
Hvel = np.array([[0.,0.,0.,1.,0.],[0.,0.,0.,0.,1.]])
Rcam = 0

filter = Filtering(Rvel, Rcam, Q, thym, Hvel, Ts)

# Begin testbench



