import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from numpy.core.fromnumeric import size
from scipy import interpolate
from copy import copy, deepcopy


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