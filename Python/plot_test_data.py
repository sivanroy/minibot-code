import numpy as np
import matplotlib.pyplot as plt
import scipy as sp
from scipy.fft import fft

from scipy import signal
def moving_average(din, n):
  # Specs:
  # din.size = dout.size
  # dout[x] = mean of din[ (x-floor(n/2)) : (x - floor(n/2) + n) ]
  # /!\ if for index x there is no sufficient data for computation, dout = 0 : important for first and last indexes
  size = len(din)
  dout = np.zeros(size)
  i = n//2
  while(i - n//2 + n < size ):
      dout[i] = np.mean(din[i-n//2:i-n//2+n])
      i+=1
  return dout

filename = "data_test_v2.txt"
Numb = 100
data = np.loadtxt(filename,delimiter = ' ',dtype=str)
array = []
print(data)

for i in range(len(data)):
    ligne = data[i]
    for j in range(len(ligne)):
        array.append(float(ligne[j]))
size = len(array)

time_tick = 587.5e-6
t = np.linspace(0,size*time_tick,size//2)
dout = np.array([array[:size//2],array[size//2:]])

mov_av = moving_average(dout[0],Numb)

num = [87.3e-3]
den = [0.873e-3,1]


ax1 = plt.subplot(111)
plt.plot(t,dout[0],alpha = 0.2,c='blue')
plt.plot(t,moving_average(dout[0],10),alpha=0.5,c='blue')
plt.plot(t,mov_av,c="green")
plt.plot(t,np.ones(size//2)*np.mean(mov_av[2600:7400]),':',c='red')
plt.plot(t,np.ones(size//2)*np.mean(mov_av[2600:7400])*.95,':',c='green')

plt.plot(t,np.ones(size//2)*np.mean(mov_av[480:1950]),':',c='red')
print(np.mean(mov_av[480:1950]),np.mean(mov_av[2600:7400]))

ax2 = ax1.twinx()
plt.plot(t,dout[1],c='orange')



plt.show()