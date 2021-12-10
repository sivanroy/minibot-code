#test_PID_controller.py
import time
import numpy as np
import matplotlib.pyplot as plt
from controller import *

from simple_pid import PID as s_PID

plot = [[],[],[],[]]

#begin scan
ref = np.ones(400)
ref[0:50]    *= 0
ref[50:100]  *= 1
ref[100:200] *= 3
ref[200:400] *= 0
m_l = np.linspace(0,4,200)
m_l = np.append(m_l,np.linspace(4,0,200))
i = 0
deltat = 1e-3
MyController = Controller(None)
Pid = s_PID(50,10,0)
Pid.output_limits = (-100,100)
Pid.sample_time=0.001

for i in range(400):
    if (i == 400):
        break
    if (i%50 == 0):
        #MyController.PID_dist.set_setpoint(ref[i])
        #MyController.PID_angle.set_setpoint(0)
        MyController.PID_speed_l.set_setpoint(ref[i])
        Pid.setpoint = ref[i]
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
    out_b = Pid(m_l_k)
    plot[3] = np.append(plot[3],out_b)

ax1 = plt.subplot(211)
plt.title("Output values")
ax1.plot(plot[2],label="out",ls=':')
ax1.plot(plot[3],label="out_b",ls="-.",alpha=0.5)
ax1.legend()
ax2 = plt.subplot(212)
plt.title("Meas;ref;error")
ax2.plot(plot[0],label='measure')
ax2.plot(plot[1],label='reference')
ax2.plot(MyController.PID_speed_l.e,label='e')
ax2.legend()
plt.show() 