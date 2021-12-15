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


plot = [[],[],[],[],[],[],[]]

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


"""
INIT = 30
STEP = 60
STEP2 = 100
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
        elif (i == 2000):
            #MyController.PID_dist.set_setpoint(ref[i])
            #MyController.PID_angle.set_setpoint(0)
            ref = STEP
            MyRobot.set_speeds(STEP,STEP)
        elif (i==4000):
            ref = STEP2
            MyRobot.set_speeds(STEP2,STEP2)


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
#plt.plot(plot[2],label="u",c='red')
plt.legend()
plt.show()

np.savetxt("data",plot[0:3])
"""




"""
MySpeedPID = PID(402.6,402.6/1.028,0,-100,100)
K = 5
MyDistPID = PID(1,0,0,-3,3)
MyAnglePID = PID(1,0,0,-3.14/2,3.14/2)

INIT = 1
STEP = 1
STEP2 = 3
ref = 0
start = time.time()


while(1):
    WhatToDo = buttonON(MyRobot)
    if (WhatToDo == -1):
        break
    elif (WhatToDo ==1):
        if (i == 0):
            MyRobot.set_speeds(0,0)
            ref = 0
        elif (i == 40000):
            MyRobot.set_speeds(0,0)
            break
        elif (i == 2000):
            #MyController.PID_angle.set_setpoint(0)
            ref = STEP
            MyAnglePID.set_setpoint(STEP)
            MyDistPID.set_setpoint(0)
        elif (i==4000):
            ref = STEP2
            MyAnglePID.set_setpoint(STEP2)
            MyDistPID.set_setpoint(0)

        m_l = MyDE02RPI.mes_left(1)
        m_r = MyDE02RPI.mes_right(1)
        #m_l_k = m_l[i]

        #D = D_odo
        D = D_wheel
        d_l = m_l * D
        d_r = m_r * D
        d_mes = (d_r+d_l)/2
        phi_mes = (d_r-d_l)/b

        #Test en BO
        #d_mes = 0
        #out_phi = 0
        out_d = MyDistPID.command(0)
        out_phi = MyAnglePID.command(0)

        omega_l_ref = out_d + out_phi 
        omega_r_ref = out_d - out_phi

        MyRobot.set_speeds(omega_l_ref,omega_r_ref)

        plot[0] = np.append(plot[0],m_l)
        plot[1] = np.append(plot[1],m_r)
        plot[2] = np.append(plot[2],ref)
        plot[3] = np.append(plot[3],phi_mes)
        i +=1

stop = time.time()
print("start : {} ; stop {} ; diff {}".format(start,stop,stop-start))

MyRobot.shutdown()
ax1 = plt.subplot(211)
#plt.plot(plot[0],label="measure",c="b")
plt.plot(moving_average(plot[0],100),label="measure mov. av. left",c='green',ls='-')
plt.plot(moving_average(plot[1],100),label="measure mov. av. right",c='green',ls='-')
#ax12 = ax1.twinx()
ax2 = plt.subplot(212)
plot[2][0] = 0
plt.plot(plot[2],label="ref",c='orange')
plt.plot(moving_average(plot[3],50),label="d measure",c='red')
plt.legend()
plt.show()

np.savetxt("data",plot[0:4]) # m_l,m_r,ref,d_mes
"""


"""
lidar = RPLidar('/dev/ttyUSB0')
lidar.start_motor()
info = lidar.get_info()
print(info)
"""




"""
MySpeedPID = PID(402.6,402.6/1.028,0,-90,90)
K = 5
#MyDistPID = PID(114.13/K,64.9/K,4.47/K,-3,3)
#MyAnglePID = PID(-13.88/5,-24.9/5,-3.11/5,-100,100)
MyDistPID = PID(500,200,0,-100,100)
MyAnglePID = PID(-10,-1,0,-100,100)

INIT = 1
STEP = .2
STEP2 = .5
ref = 0
ref_a = 0
start = time.time()
d_passed_mes = []
a_passed_mes =[]
passed = 10
i = 0


while(1):
    WhatToDo = buttonON(MyRobot)
    if (WhatToDo == -1):
        break
    elif (WhatToDo ==1):
        if (i == 0):
            MyRobot.set_speeds(0,0)
            ref = 0
        elif (i == 40000):
            MyRobot.set_speeds(0,0)
            break
        elif (i == 2000):
            #MyController.PID_angle.set_setpoint(0)
            ref = STEP
            print(ref)
            MyAnglePID.set_setpoint(0)
            MyDistPID.set_setpoint(STEP)
        elif (i==4000):
            ref = STEP2
            MyAnglePID.set_setpoint(0)
            MyDistPID.set_setpoint(STEP2)

        m_l = MyDE02RPI.mes_left(1)
        m_r = MyDE02RPI.mes_right(1)
        #m_l_k = m_l[i]

        #D = D_odo
        b = 0.2345 
        D = D_wheel
        d_l = m_l * D
        d_r = m_r * D
        d_mes = (d_r+d_l)/2
        phi_mes = (d_r-d_l)/b

        #to removz
        #print(len(d_passed_mes))
        if (len(d_passed_mes) < passed):
            d_passed_mes.append(d_mes)
            a_passed_mes.append(phi_mes)
        else :
            if(i >= passed):
                i=0
            #print(i)
            d_passed_mes[i] = d_mes
            a_passed_mes[i] = d_mes

        d_mean = 0
        a_mean = 0
        for j in range (len(d_passed_mes)):
            d_mean+=d_passed_mes[j]
            a_mean+=a_passed_mes[j]
        i+=1
        #to remove

        #Test en BF
        out_d = MyDistPID.command(d_mes)#an/passed)
        out_phi = MyAnglePID.command(phi_mes)#an/passed)
        omega_l_ref = out_d + out_phi 
        omega_r_ref = out_d - out_phi
        MyRobot.set_speeds(omega_l_ref,omega_r_ref)

        plot[0] = np.append(plot[0],m_l)
        plot[1] = np.append(plot[1],m_r)
        plot[2] = np.append(plot[2],ref)
        plot[3] = np.append(plot[3],d_mes)
        plot[4] = np.append(plot[4],phi_mes)
        i +=1

stop = time.time()
print("start : {} ; stop {} ; diff {}".format(start,stop,stop-start))

"""


"""
lidar.stop()
lidar.stop_motor()
lidar.disconnect()
"""

MyRobot.shutdown()
ax1 = plt.subplot(211)
#plt.plot(plot[0],label="measure",c="b")
plt.plot(moving_average(plot[0],100),label="measure mov. av. left",c='green',ls='-')
plt.plot(moving_average(plot[1],100),label="measure mov. av. right",c='green',ls='-')
#ax12 = ax1.twinx()
ax2 = plt.subplot(212)
plot[2][0] = 0
plt.plot(plot[2],label="ref",c='orange')
plt.plot(moving_average(plot[3],50),label="d measure",c='red')
plt.legend()
#ax3 = ax2.twinx()
#plt.plot(plot[3])
plt.show()

np.savetxt("data",plot[0:4]) # m_l,m_r,ref,d_mes



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