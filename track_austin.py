import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from numpy.core.fromnumeric import size
from scipy import interpolate
import math
import time
 
## Use this function to calculate reference path's x, y, and heading angle theta
def ref_traj():
    file = open("Austin.csv")
    data = np.loadtxt(file,delimiter=",")
    x_pos = data[:,0]
    y_pos = data[:,1]
    #slope = np.zeros(len(x_pos))
    heading = np.zeros(len(x_pos))
    for i in range(len(x_pos)-1):
        heading[i] = math.atan2( (y_pos[i+1]-y_pos[i]),(x_pos[i+1]-x_pos[i]))
    
    #heading = math.atan2(slope)
    return x_pos,y_pos,heading

def SearchMinDistance(x,y,x_ref,y_ref):
    dis = np.zeros(len(x_ref))
    min_dis = 100000
    for i in range(len(dis)-1):
        dis[i] = (x-x_ref[i])**2+(y-y_ref[i])**2
        if dis[i]<min_dis:
            min_dis = dis[i]
    return min_dis

def dynamics(x,y,v,theta,a,theta_dot):
    x_next = 0
    y_next = 0
    v_next = 0
    theta_next = 0
    dt = 0.1 # dt is small time interval that requires for discretize bicycle model

    x_next = x + v*np.cos(theta)*dt
    y_next = y + v*np.sin(theta)*dt
    v_next = v + a*dt
    theta_next = theta + theta_dot*dt

    return x_next, y_next,v_next,theta_next


[x_ref,y_ref,theta_ref]=ref_traj()

t_start = time.time()
time_span = 30
dt = 0.1
Kp = 1
Kd = 1
# for i in range(time_span/dt):
    
#     v = 10
#     accel = 0
#     x_current = 10
#     y_current = 10
#     theta_current = 0
#     theta_dot = 