#import serial as sr
from serial import Serial
import matplotlib.pyplot as plt
import numpy as np


s = Serial('COM5',9600)
plt.close()
plt.figure()
plt.ion()
plt.show()

data = np.array([])
i = 0
while i<10000:
    a = s.readline()
    a.decode()
    b = float(a[0:4]);
    data = np.append(data,b)
    plt.cla()
    plt.plot(data)
    plt.pause(0.01)
    i = i+1