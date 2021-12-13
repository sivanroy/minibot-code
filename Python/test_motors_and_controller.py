from rplidar import RPLidar
import RPi.GPIO as GPIO
import time
import numpy as np
import headquarters as h
from robot import *
from controller import *
import matplotlib.pyplot as plt

#begin motors
#motors.start_all()

#begin lidars


MyRobot = Robot()

def buttonON(MyRobot):
    Mybutton1 = MyRobot.sensors.buttons.button1 #gere le button
    if (Mybutton1.wasPushed() == 0):
        if(MyRobot.isON()):
            return 1
    else:
        if(Mybutton1.count()<2):
            Mybutton1.clear()
            MyRobot.activate()
            print(Mybutton1.print_infos())
            print("Robot is ON\n")
            
        else:
            Mybutton1.clear()
            MyRobot.shutdown()
            MyRobot.actuators.motors.set_speeds(0,0)
            print("stop the motors \n")
            return -1 
    return 0

def moving_average(din, n):
  # Specs:
  # din.size = dout.size
  # dout[x] = mean of din[ (x-floor(n/2)) : (x - floor(n/2) + n) ]
  # /!\ if for index x there is no sufficient data for computation, dout = 0 : important for first and last indexes
  size = len(din)
  print(size)
  dout = np.zeros(size)
  i = n//2
  while(i - n//2 + n < size ):
      dout[i] = np.mean(din[i-n//2:i-n//2+n])
      i+=1
  return dout


plot = [[],[],[]]

#begin scan
ref = np.ones(4000)
ref[0:500]    *= 0
ref[500:1000]  *= 1
ref[1000:2000] *= 3
ref[2000:4000] *= 0


i = 0
deltat = 1e-3
MyController = MyRobot.controller
MyDE02RPI = MyRobot.controller.DE02RPI

mean = 0
INIT = 30
STEP = 60
ref = 0
start = time.time()
while(1):
    WhatToDo = buttonON(MyRobot)
    if (WhatToDo == -1):
        break
    elif (WhatToDo ==1):
        if (i == 0):
            MyRobot.set_speeds(INIT,INIT)
            ref = INIT
        elif (i == 40000):
            MyRobot.set_speeds(0,0)
            break
        elif (i%2000 == 0 and i!=0):
            #MyController.PID_dist.set_setpoint(ref[i])
            #MyController.PID_angle.set_setpoint(0)
            ref = STEP
            MyRobot.set_speeds(STEP,STEP)

        m_l = MyDE02RPI.mes_left(1)
        m_r = MyDE02RPI.mes_right(1)
        #m_l_k = m_l[i]

        #m_r = MyDE02RPI.mes_right(1)
        #MyController.set_measures(m_l,m_r)
        plot[0] = np.append(plot[0],m_l)
        plot[1] = np.append(plot[1],ref)
        #plot[2] = np.append(plot[2],output)
        i +=1
stop = time.time()
print("start : {} ; stop {} ; diff {}".format(start,stop,stop-start))

MyRobot.shutdown()
ax1 = plt.subplot(111)
#plt.plot(plot[0],label="measure",c="b")
plt.plot(moving_average(plot[0],100),label="measure mov. av.",c='green',ls='-')
ax12 = ax1.twinx()
plot[1][0] = 0
plt.plot(plot[1],label="ref",c='orange')
plt.legend()
plt.show()

"""
mean = 0
while(1):
    WhatToDo = buttonON(MyRobot)
    if (WhatToDo == -1):
        break
    elif (WhatToDo ==1):
        if (i == 4000):
            break
        if (i%500 == 0):
            #MyController.PID_dist.set_setpoint(ref[i])
            #MyController.PID_angle.set_setpoint(0)
            MyController.PID_speed_l.set_setpoint(ref[i])
            print(ref[i])

        #m_l = MyDE02RPI.mes_left(1)
        #m_r = MyDE02RPI.mes_right(1)
        m_l_k = m_l[i]

        #m_r = MyDE02RPI.mes_right(1)
        #MyController.set_measures(m_l,m_r)
        plot[0] = np.append(plot[0],m_l)
        plot[1] = np.append(plot[1],ref[i])
        u_r,u_l = MyController.compute_all_chain(1)
        plot[2] = np.append(plot[2],output)
        MyRobot.set_speeds(out,0)
"""

MyRobot.shutdown()
plt.plot(plot[0],label="measure")
plt.plot(plot[1],label="ref")
plt.plot(plot[2],label="output")
plt.legend()
#plt.show()