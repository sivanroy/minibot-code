import numpy as np
import RPi.GPIO as GPIO
from robot import *
from displacement import *
from controller import *
import time

MAXDIST = 600
MINDIST = 300
ERROR = 20
speed = 1

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

i = 0 
def closed_loop_ref(scan_data,MyRobot):
    MyController = MyRobot.controller
    if (i<100):
        MyController.set_cartesian_ref(1,1)
        i+=1

def closed_loop_omega(scan_data,MyRobot):
    MyController = MyRobot.controller
    PID_obj = MyController.PID_obj
    if (i<100):
        PID_obj.set_setpoint_l(1)
        PID_obj.set_setpoint_r(1)
        i+=1


def headqarters(scan_data,MyRobot):
    Mybutton1 = MyRobot.sensors.buttons.button1 #gere le button
    if (Mybutton1.wasPushed() == 0):
        if(MyRobot.isON()):
            follow(scan_data,MyRobot)
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
    return 1

    