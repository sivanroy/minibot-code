import numpy as np
import RPi.GPIO as GPIO
from robot import *
from displacement import *
from controller import *
from FIFO import *
#from button import buttonOn
import time

c = 4
MAXDIST = 1000
MINDIST = 300
ERROR = 20
speed = 1
index = 0
deltat = 2e-3

PID_dist =  PID(30,10,0,-1000,1000)
PID_angle = PID(3,0.02,0,-1000,1000)
#fifo_l = FIFO(1)
#fifo_r = FIFO(1)


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


"""
Open loop function base on scan_data
Folow the closer beacon
"""
def follow(scan_data,MyRobot):
    dist = scan_data[0]
    theta = scan_data[1]
    motors = MyRobot.actuators.motors
    max_index = np.where(dist == np.amin(np.array(dist)))[0][0]
    dist_p = dist[max_index]
    theta_p = theta[max_index]
    # Balise trop proche -> stop
    if (dist_p < MINDIST or dist_p > MAXDIST):
        motors.stop_all()
    # Balise à gauche
    elif (theta_p < 180 - ERROR):
        # Balise arriere gauche
        if (theta_p < 90 - ERROR):
            motors.set_speeds(-speed, speed)
        # Balise avant gauche
        else:
            motors.set_speeds(0, speed)
    # Balise à droite
    elif (theta_p > 180 + ERROR):
        # Balise arriere droite
        if (theta_p > 270 - ERROR):
            motors.set_speeds(speed, -speed)
        # Balise avant gauche
        else:
            motors.set_speeds(speed, 0)
    # Balise en face
    else:
        motors.set_speeds(speed, speed)


def dodge(scan_data):
	return 1

def test_theta_ref(scan_data,MyRobot):
    SLEEP = 1
    ref = [0,1,2,0.05,0]
    for value in ref:
        MyRobot.controller.set_ref_polar(value,0,1)
        print(value)
        if (SLEEP): 
            time.sleep(5)
            print("sleep")

"""

"""
def closed_loop_omega(scan_data,MyRobot):
    MyController = MyRobot.controller
    PID_obj = MyController.PID_obj
    i = 0
    if (i<100):
        PID_obj.set_setpoint_l(i)
        PID_obj.set_setpoint_r(i)
        i+=1


"""
theta / dist reference
"""
"""
def closed_loop_ref(scan_data,MyRobot
    ):
    verbose = 0
    dist_p, theta_p = closer(scan_data,MyRobot)
    theta_p -= np.pi/2

    if(dist_p < 300):
        MyRobot.controller.set_polar(0,0)
        if (verbose) :print("To close")
    else:
        if(index<100): 
            index += 1
        else:
            MyRobot.controller.set_polar(dist_p/1000,theta_p)
            index = 0

    if(verbose): print(MyRobot.controller.PID_obj.omega_mes_l)
"""


def closed_loop(scan_data,MyRobot):
    d_p,theta_p = closer(scan_data)
    #K_p_d = 1
    #K_p_a = 0.01
    MyController = MyRobot.controller
    MyDE02RPI = MyController.DE02RPI
    #if(d_p > MINDIST and d_p < MAXDIST):
    d_ref = (d_p-MINDIST)/1000
    theta_ref = (theta_p-180)*3.14/180

    m_l = MyDE02RPI.mes_left(1)
    m_r = MyDE02RPI.mes_right(1)

    b = 0.2345 
    D = D_wheel

    d_l = m_l * D
    d_r = m_r * D
    d_mes = (d_r+d_l)/2
    phi_mes = (d_r-d_l)/b

    do = 1
    #if(abs(d_ref-d_mes)<0.1):
    #    do = 0
    #    print("to close")
    #P
    PID_dist.set_setpoint(d_ref)
    print("theta-phi ref \n")
    dout =  PID_dist.command(d_mes,1)
    #if (do==0):
    #    dout = 0
    #dout = K_p_d*(d_ref-d_mes)
    PID_angle.set_setpoint(theta_ref)
    alphaout = PID_angle.command(phi_mes,1)
    #alphaout = K_p_a*(theta_ref-phi_mes)
    sp_l = dout+alphaout
    sp_r = dout-alphaout
    #setpoint
    MyController.PID_speed_l.set_setpoint(sp_l)
    MyController.PID_speed_r.set_setpoint(sp_r)
    
    #take out and sent to wheels
    print("m_l m_r\n")
    out_l = MyController.PID_speed_l.command(m_l,1)
    out_r = MyController.PID_speed_r.command(m_r,1)

    #fifo_l.push(out_l)
    #fifo_r.push(out_r)

    #MyRobot.set_speeds(fifo_l.get_mean(),fifo_r.get_mean())
    MyRobot.set_speeds(out_l,out_r)
    
    print("\n[{};{}]\n".format(out_l , out_r))
    #print("[{};{}]".format(d_ref , theta_ref))
    #print("[{};{}]".format(dout , alphaout))
    #print("d_mes {}:: d_ref{}\nphi_mes{}::phi_ref{}".\
        #format(d_mes,d_ref,phi_mes,theta_ref))
    #print(d_p,theta_p)


    """else:
        MyRobot.controller.set_ref_polar(0,0)
        MyRobot.set_speeds(0,0)
        print("to close")
"""


def headquarters(scan_data,MyRobot):
    c = 4
    WhatToDo = buttonON(MyRobot)
    if (WhatToDo == -1):
        MyRobot.shutdown()
        return -1
    elif(WhatToDo == 1):
        if  (c == 0): 
            follow(scan_data,MyRobot)
        #elif(c == 1): closed_loop_ref(scan_data,MyRobot)
        elif(c == 2): 
            closed_loop_omega(scan_data,MyRobot)
        elif(c == 3): 
            test_theta_ref(scan_data,MyRobot)
        elif(c == 4):
            closed_loop(scan_data,MyRobot)  
        else:
            print("nothing to do in headquarters")
    return 1
