import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from numpy.core.fromnumeric import size
from scipy import interpolate
from copy import copy, deepcopy

class KalmanFilter(object):
    def __init__(self, dt, u, std_acc, std_meas):
        self.dt = dt
        self.u = u
        self.std_acc = std_acc

        self.A = np.matrix([[1, self.dt],
                            [0, 1]])
        self.B = np.matrix([[(self.dt**2)/2], [self.dt]])

        self.H = np.matrix([[1, 0]])

        self.Q = np.matrix([[(self.dt**4)/4, (self.dt**3)/2],
                            [(self.dt**3)/2, self.dt**2]]) * self.std_acc**2

        self.R = std_meas**2

        self.P = np.eye(self.A.shape[1])
        
        self.x = np.matrix([[0], [0]])

        


   



def main():
    
    # creating the main track 
    file = open("Austin.csv")
    real_track = np.loadtxt(file, delimiter=",")

    # creating fake gps noise 
    fake_gps = deepcopy(real_track)
    
    fake_gps[:,0] = real_track[:,0] +  np.random.normal(0,50)
    fake_gps[:,1] = real_track[:,1] +  np.random.normal(0,50)



    # plotting x and y coordinates of real_track and gps
    plt.plot(real_track[:,0],real_track[:,1], '--r')
    plt.plot(fake_gps[:,0], fake_gps[:,1], '-b')
    plt.show()

if __name__ == '__main__':
    main()