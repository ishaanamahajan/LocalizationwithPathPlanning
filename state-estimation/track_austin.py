import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from numpy.core.fromnumeric import size
from scipy import interpolate

 
#track_data = pd.read_csv(r'C:\Users\lenovo\Documents\ME468\FinalProject\racetrack-database\tracks\Austin.csv')
#print(track_data)
file = open("Austin.csv")
data = np.loadtxt(file,delimiter=",")
print(data)
#plt.plot(data[:,0],data[:,1])
#plt.show()