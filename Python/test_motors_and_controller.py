from rplidar import RPLidar
import RPi.GPIO as GPIO
import time
import numpy as np
import headquarters as h
from robot import *

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
        #MyController.compute_all_chain(1)
        out = MyController.PID_speed_l.output_value(mean,1)
        plot[2] = np.append(plot[2],output)
        MyRobot.set_speeds(out,0)


MyRobot.shutdown()
plt.plot(plot[0],label="measure")
plt.plot(plot[1],label="ref")
plt.plot(plot[2],label="output")
plt.legend()
plt.show()