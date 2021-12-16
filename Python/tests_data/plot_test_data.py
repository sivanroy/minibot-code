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

time_tick = 587.5e-6


#OPEN LOOP Angle
filename = "data_BF_theta_dnul.txt" #ml;mr;ref;phimes
Numb = 100
data = np.loadtxt(filename,delimiter = ' ',dtype=str)
array = []

for i in range(len(data)):
    ligne = data[i]
    for j in range(len(ligne)):
        array.append(float(ligne[j]))
size = len(array)//4

t = np.linspace(0,size*time_tick,size)
dout = np.array([array[:size],array[size:2*size],array[size*2:size*3],array[size*3:]])
mov_av = moving_average(dout[3],Numb)

ax1 = plt.subplot(111)
#plt.plot(t,dout[3],alpha = 0.2,c='blue',label="d meas.")
plt.title("Measure of theta in open loop")
plt.plot(t,moving_average(dout[3],10),alpha=0.3,c='blue',label = "d meas. #10")
plt.plot(t,mov_av,c="green",label = "d meas.#100")
plt.plot(t,np.ones(size)*np.mean(mov_av[2500:3500]),':',c='red')
plt.plot(t,np.ones(size)*np.mean(mov_av[2500:3500])*.95,':',c='green')
plt.plot(t,np.ones(size)*np.mean(mov_av[4400:15000]),':',c='red')
plt.plot(t,np.ones(size)*np.mean(mov_av[4400:15000])*.95,':',c='green')
plt.legend()

print(np.mean(mov_av[2500:3500]),np.mean(mov_av[4400:15000])) #-0.23955219635261127 -0.27378180497494126
ax11 = ax1.twinx()
plt.plot(t,dout[2], label="reference", c="orange")
plt.legend()
plt.show()

#OPEN LOOP Distance
filename = "data_BF_d_thetanul.txt" #ml;mr;ref;phimes
Numb = 100
data = np.loadtxt(filename,delimiter = ' ',dtype=str)
array = []

for i in range(len(data)):
    ligne = data[i]
    for j in range(len(ligne)):
        array.append(float(ligne[j]))
size = len(array)//4

t = np.linspace(0,size*time_tick,size)
dout = np.array([array[:size],array[size:2*size],array[size*2:size*3],array[size*3:]])
mov_av = moving_average(dout[3],Numb)

ax1 = plt.subplot(111)
#plt.plot(t,dout[3],alpha = 0.2,c='blue',label="d meas.")
plt.title("Measure of d in open loop")
plt.plot(t,moving_average(dout[3],10),alpha=0.5,c='blue',label = "d meas. #10")
plt.plot(t,mov_av,c="green",label = "d meas.#100")
plt.legend()
plt.plot(t,np.ones(size)*np.mean(mov_av[2300:3300]),':',c='red')
plt.plot(t,np.ones(size)*np.mean(mov_av[2300:3300])*.95,':',c='green')
plt.plot(t,np.ones(size)*np.mean(mov_av[5200:12000]),':',c='red')
plt.plot(t,np.ones(size)*np.mean(mov_av[5200:12000])*.95,':',c='green')
print(np.mean(mov_av[2300:3300]),np.mean(mov_av[5200:12000])) # 0.02576029276904236 -0.02659340675835945
ax11 = ax1.twinx()
plt.plot(t,dout[2], label="reference", c="orange")
plt.legend()
plt.show()

"""
ax2 = plt.subplot(212)
plt.plot(t,dout[0],c='orange', label = "left meas.")
plt.plot(t,dout[1],c='purple',label = "right meas.")
plt.legend()
plt.show()
"""

#OPEN LOOP MOTOR SPEED
filename = "data_test_v2.txt"
Numb = 100
data = np.loadtxt(filename,delimiter = ' ',dtype=str)
array = []

for i in range(len(data)):
    ligne = data[i]
    for j in range(len(ligne)):
        array.append(float(ligne[j]))
size = len(array)

t = np.linspace(0,size*time_tick,size//2)
dout = np.array([array[:size//2],array[size//2:]])
mov_av = moving_average(dout[0],Numb)
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
