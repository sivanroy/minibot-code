import numpy as np
import RPi.GPIO as GPIO
from robot import *
from displacement import *
from controller import *
#from button import buttonOn
import time

c = 4
MAXDIST = 1000
MINDIST = 300
ERROR = 20
D_wheel = 0.06          #wheels diameter 


PID_speed_l = PID(100,1,0,-100,100)
PID_speed_r = PID(100,1,0,-100,100)
PID_dist =  PID(1000,0,0)
PID_angle = PID(200,0,0)

x_shared = [0]
y_shared = [0]
theta_shared = [0]
i = 0

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


"""
closer scan_data
"""
def closer(scan_data):
    dist = scan_data[0]
    theta = scan_data[1]
    max_index = np.where(dist == np.amin(np.array(dist)))[0][0]
    dist_p = dist[max_index]
    theta_p = theta[max_index]
    return dist_p,theta_p

def compute_d_phi(m_l,m_r):
    b = 0.2345 
    D = D_wheel
    d_l = m_l * D
    d_r = m_r * D
    d_mes = (d_r+d_l)/2
    phi_mes = (d_r-d_l)/b
    return d_mes,phi_mes

def upsate_pos(d_mes, phi_mes, ind):
    x_new = x[ind] + d_mes * np.cos(theta[ind] + phi_mes / 2)
    y_new = y[ind] + d_mes * np.sin(theta[ind] + phi_mes / 2)
    theta_new = theta[ind] + phi_mes
    x.append(x_new)
    y.append(y_new)
    theta.append(theta_new)
    ind += 1

def closed_loop(scan_data,MyRobot):
    d_p,theta_p = closer(scan_data)
    MyController = MyRobot.controller
    MyDE02RPI = MyController.DE02RPI

    d_ref = (d_p-MINDIST)/1000
    theta_ref = (theta_p-180)*3.14/180

    m_l = MyDE02RPI.mes_left(1)
    m_r = MyDE02RPI.mes_right(1)

    d_mes,phi_mes = compute_d_phi(m_l,m_r)

    #PID phi et d
    PID_dist.set_setpoint(d_ref)
    dout =  PID_dist.command(d_mes)
    PID_angle.set_setpoint(theta_ref)
    alphaout = PID_angle.command(phi_mes)
    sp_l = dout+alphaout
    sp_r = dout-alphaout

    #setpoint of wheels
    PID_speed_l.set_setpoint(sp_l)
    PID_speed_r.set_setpoint(sp_r)
    
    #take out and sent to wheels
    out_l = PID_speed_l.command(m_l)
    out_r = PID_speed_r.command(m_r)

    MyRobot.set_speeds(out_l,out_r)

    #print("[{};{}]".format(d_ref , theta_ref))
    print("[{};{}]".format(dout , alphaout))
    #print("d_mes {}:: d_ref{}\nphi_mes{}::phi_ref{}".format(d_mes,d_ref,phi_mes,theta_ref))
    #print(d_p,theta_p)


def headquarters(scan_data,MyRobot):
    WhatToDo = buttonON(MyRobot)
    if (WhatToDo == -1):
        MyRobot.shutdown()
        return -1
    elif(WhatToDo == 1):
        closed_loop(scan_data,MyRobot)  
    return 1
