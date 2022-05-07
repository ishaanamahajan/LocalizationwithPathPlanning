#Built on a kalman filter previously implemented by Ishaan Mahajan using a tutorial 

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

    def predict(self):
       
        self.x = np.dot(self.A, self.x) + np.dot(self.B, self.u)

        
        self.P = np.dot(np.dot(self.A, self.P), self.A.T) + self.Q
        return self.x

    def update(self, z):
        
        S = np.dot(self.H, np.dot(self.P, self.H.T)) + self.R

        
        K = np.dot(np.dot(self.P, self.H.T), np.linalg.inv(S))  

        self.x = np.round(
            self.x + np.dot(K, (z - np.dot(self.H, self.x))))  

        I = np.eye(self.H.shape[1])
        self.P = (I - (K * self.H)) * self.P  


def main():

    initial_vel = 2
    dt = 0.1
    std_acc = 0.8
    error = 1.2

    # creating the main track
    file = open("Austin.csv")
    real_track = np.loadtxt(file, delimiter=",")

    # creating fake gps noise
    fake_gps = deepcopy(real_track)

    fake_gps[:, 0] = real_track[:, 0] + np.random.normal(0, 150)
    fake_gps[:, 1] = real_track[:, 1] - np.random.normal(0, 150)

    kf = KalmanFilter(dt, initial_vel, std_acc, error)
    predictions = []

    for x in real_track[:, 0]:
    
        predictions.append(kf.predict()[0])
        #predictions[1].append(kf.predict()[0])
        #predictions[:, 1] = kf.predict()[0]


    print(predictions)

    # plotting x and y coordinates of real_track and gps
    plt.plot(real_track[:, 0], real_track[:, 1], '-b', label = "Actual track")
    plt.plot(fake_gps[:, 0], fake_gps[:, 1], '--r', label = "GPS data (with noise)")
    plt.plot(np.squeeze(predictions) , real_track[: , 0], '-g', label = "Kalman Filter Estimation")
    plt.plot(np.squeeze(predictions) , real_track[: , 1], '-g')
    plt.legend()
    plt.xlabel("Latitude")
    plt.ylabel("Longitude")
    plt.title("Kalman Filter Implementation")
    
    plt.show()


if __name__ == '__main__':
    main()
