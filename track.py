import numpy as np
import matplotlib.pyplot as plt
from numpy.core.fromnumeric import size
from scipy import interpolate

 

timestamp = []
distance = []


xInner = [49.8, 60.3 ,	75.6, 	87.9,	96.9,	111.0,	115.2,	120.6,	127.8,	135.9, 135.9, 	133.2,	128.4,	119.7,	105.0,	90.0,	82.5,	82.5,	83.4,	77.1,	61.2,	55.5,	57.9,	66.6	,75.9,	79.2,	78.0,	65.1, 50.7,	36.6	,29.1 ,24.0,	24.0 ,29.1,	24.9,	13.5,	6.3	,5.7,	6.3	,8.7,	15.3,	24.3,	31.2,	40.8,	49.8 ]
yInner = [132.9	,129.3	,129.0	,131.7	,129.6	,120.0	,110.7	,96.9	,88.5	,77.4	,65.1	,51.3	,43.2	,36.3	,35.7	,36.3	,46.2	,63.6	,82.2	,93.9	,88.5	,73.5	,54.6	,45.0	,36.3	 ,   25.5	 ,13.2,	6.0	,6.0	,11.7	,21.3	,36.9	,56.1	,70.8	 ,77.7	,77.7	,81.6	,92.7	,107.7	 ,118.2	,122.7	,125.4	,126.0	,129.6	,132.9]

for i in range(len(xInner)-1 ):
    timestamp.append(i)


#plotting the path
plt.plot(xInner, yInner)
plt.show()




 

