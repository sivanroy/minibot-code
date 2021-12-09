import numpy as np
import RPi.GPIO as GPIO
from robot import *
from displacement import *
from controller import *
#from button import buttonOn
import time

MAXDIST = 600
MINDIST = 300
ERROR = 20
speed = 1

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
def closer(scan_data,MyRobot):
    dist = scan_data[0]
    theta = scan_data[1]
    motors = MyRobot.actuators.motors
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
    ref = np.ones((2,1000))
    ref[0][0:100] = 0
    ref[0][100:200] = 1
    ref[0][200:300] = -1
    ref[0][300:400] = 2
    ref[0][400:500] = -2
    ref[0][500:600] = 0.05
    ref[0][600:700] = -0.05

    for values in ref:
        dist = values[0]
        alpha = values[1]
        MyRobot.controller.set_polar(dist,alpha)

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
def closed_loop_ref(scan_data,MyRobot):
    verbose = 0
    dist_p, theta_p = closer(scan_data,MyRobot)
    theta_p -= np.pi/2

    if(dist_p < 300):
        MyRobot.controller.set_polar(0,0)
        if (verbose) :print("To close")
    else:
        if(i<100): 
            i += 1
        else:
            MyRobot.controller.set_polar(dist_p/1000,theta_p)
            i = 0

    if(verbose): print(MyRobot.controller.PID_obj.omega_mes_l)



def headquarters(scan_data,MyRobot):
    c = 1
    WhatToDo = buttonON(MyRobot)
    if (WhatToDo == -1):
        MyRobot.shutdown()
        return -1
    elif(WhatToDo == 1):
        if  (c == 0): follow(scan_data,MyRobot)
        elif(c == 1): closed_loop_ref(scan_data,MyRobot)
        elif(c == 2): closed_loop_omega(scan_data,MyRobot)
        else: print("Not valid")
    return 1
