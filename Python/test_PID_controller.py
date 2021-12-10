#test_PID_controller.py
import time
import numpy as np
import matplotlib.pyplot as plt
from controller import *


plot = [[],[],[]]

#begin scan
ref = np.ones(400)
ref[0:50]    *= 0
ref[50:100]  *= 1
ref[100:200] *= 3
ref[200:400] *= 0
m_l = np.linspace(0,2,200)
m_l = np.append(m_l,np.linspace(2,0,200))
i = 0
deltat = 1e-3
MyController = Controller(None)

for i in range(400):
    if (i == 400):
        break
    if (i%50 == 0):
        #MyController.PID_dist.set_setpoint(ref[i])
        #MyController.PID_angle.set_setpoint(0)
        MyController.PID_speed_l.set_setpoint(ref[i])
        print(ref[i])

    #m_l = MyDE02RPI.mes_left(1)
    #m_r = MyDE02RPI.mes_right(1)
    m_l_k = m_l[i]

    #m_r = MyDE02RPI.mes_right(1)
    #MyController.set_measures(m_l,m_r)
    plot[0] = np.append(plot[0],m_l_k)
    plot[1] = np.append(plot[1],ref[i])
    #MyController.compute_all_chain(1)
    out = MyController.PID_speed_l.command(m_l_k,0)
    plot[2] = np.append(plot[2],out)


plt.plot(plot[0],label="measure")
plt.plot(plot[1],label="ref")
plt.plot(plot[2]/100,label="output")
plt.legend()
plt.show() 