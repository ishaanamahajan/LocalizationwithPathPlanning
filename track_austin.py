import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from numpy.core.fromnumeric import size
from scipy import interpolate

 
#track_data = pd.read_csv(r'C:\Users\lenovo\Documents\ME468\FinalProject\racetrack-database\tracks\Austin.csv')
#print(track_data)
file = open("Austin.csv")
data = np.loadtxt(file,delimiter=",")
#print(data)
#plt.plot(data[:,0],data[:,1])
#plt.show()
x_pos = data[:,0]
y_pos = data[:,1]
slope = np.zeros(len(x_pos))
for i in range(len(x_pos)-1):
    slope[i] = (y_pos[i+1]-y_pos[i])/(x_pos[i+1]-x_pos[i])


