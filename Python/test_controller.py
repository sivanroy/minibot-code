#test_PID_controller.py
import time
import numpy as np
import matplotlib.pyplot as plt
from controller import *

plot = [[],[],[],[],[]]

#begin scan
ref = np.ones(400)
ref[0:50]    *= 0
ref[50:100]  *= 1
ref[100:200] *= 3
ref[200:400] *= 0

m_l = np.linspace(0,4,200)
m_l = np.append(m_l,np.linspace(4,0,200))
m_r = m_l
i = 0
deltat = 1e-3
MyController = Controller(None)


for i in range(400):
    if (i == 400):
        break
    if (i%50 == 0):
        #MyController.PID_dist.set_setpoint(ref[i])
        #MyController.PID_angle.set_setpoint(0)
        MyController.PID_dist.set_setpoint(ref[i])
        MyController.PID_angle.set_setpoint(0)
        print(ref[i])

    #m_l = MyDE02RPI.mes_left(1)
    #m_r = MyDE02RPI.mes_right(1)
    m_l_k = m_l[i]
    m_r_k = m_r[i]
    MyController.set_measures(m_l_k,m_r_k)
    plot[0].append(m_l_k)
    plot[1].append(m_r_k)
    plot[2].append(ref[i])
    ur,ul = MyController.compute_all_chain(1,verbose=0)
    plot[3].append(ul)
    plot[4].append(ur)


ax1 = plt.subplot(211)
plt.title("Output values")
ax1.plot(plot[3],label="u_l",ls=':')
ax1.plot(plot[4],label="u_r",ls="-.")
ax1.legend()
ax2 = plt.subplot(212)
plt.title("Meas;ref;error")
ax2.plot(plot[0],label='measure l')
ax2.plot(plot[1],label='measure r')
ax2.plot(plot[2],label='reference')
ax2.plot(MyController.PID_dist.e,label='e')
ax2.legend()
plt.show()