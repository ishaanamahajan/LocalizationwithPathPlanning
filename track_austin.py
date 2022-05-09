from cProfile import label
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from numpy.core.fromnumeric import size
from scipy import interpolate
import math
import time
 
## Below it is to calculate reference path's x, y, and heading angle theta
## We name it as x_ref, y_ref and theta_ref

file = open("path1.csv")
data = np.loadtxt(file,delimiter=",")
x_ref = data[:,0]
y_ref = data[:,1]
        #slope = np.zeros(len(x_ref))
theta_ref = np.zeros(len(x_ref))
for i in range(len(x_ref)-1):
    theta_ref[i] = -np.pi+ math.atan2( (y_ref[i+1]-y_ref[i]),(x_ref[i+1]-x_ref[i]))

def SearchMinDistance(x,y):
    dis = np.zeros(len(x_ref))
    min_dis = 100000
    index = -1
    for i in range(len(dis)-1):
        dis[i] = (x-x_ref[i])**2+(y-y_ref[i])**2
        if dis[i]<min_dis:
            min_dis = dis[i]
            index = i
    return min_dis,index

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

def Direction(theta_ref,theta,theta_dot):
    if theta > theta_ref:
        theta_dot = -theta_dot
    else:
        theta_dot = theta_dot
    return theta_dot

    
# x = 200
# y = -40
t_start = time.time()
time_span = 30
dt = 0.1
Kp = 40
Kd = 5
v = 0.4
accel = 0
x_current = 5
y_current = 3
theta_current = 0
theta_dot_current = 0
min_dis_pre = 0
traj_x = [x_current]
traj_y = [y_current]
for i in range(500):
    #initial condition setting

    # calculating theta_dot, the one of input given in this step. Here we adjust theta_dot based on Kp term
    [min_dis,index] = SearchMinDistance(x_current,y_current)
    theta_dot_raw = Kp * min_dis
    theta_dot_current = Direction(theta_ref[index],theta_current,theta_dot_raw)
    
    # calculating theta_dot, the one of input given in this step. Here we adjust theta_dot based on Kd term
    theta_dot_current = theta_dot_current*Kd*(min_dis-min_dis_pre)
    min_dis_pre = min_dis
    # plug in theta_dot into our bicycle dynamics model
    [x_current,y_current,v,theta_current] = dynamics(x_current,y_current,v,theta_current,accel,theta_dot_current)
    #print(x_current,y_current,v,theta_current,theta_dot_current)
    traj_x.append(x_current)
    traj_y.append(y_current)
    #plt.plot(x_current,y_current,'r*-',label='Actual Trajectory')


#print(theta_ref[index],theta_current)
#plt.plot(10,10,'ro')
plt.plot(traj_x,traj_y,'r*-',label='Actual Trajectory')
plt.plot(x_ref,y_ref,label='Reference Trajectory')
plt.legend()
plt.show()
