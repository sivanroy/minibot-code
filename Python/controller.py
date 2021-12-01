import displacement
import numpy as np
import RPi.GPIO as GPIO
from robot import *
from displacement import *
import time

MAXDIST = 600
MINDIST = 300
ERROR = 20
speed = 0.5

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

def controller(scan_data,MyRobot):
    Mybutton1 = MyRobot.sensors.buttons.button1 #gere le button
    if (Mybutton1.isPushed() == 0):
        return 1
    else:
        if(Mybutton1.count()<2):
            Mybutton1.clear()
            MyRobot.activate()
            print(Mybutton1.print_infos())
            print("Robot is ON\n")
            return 1
        else:
            Mybutton1.clear()
            MyRobot.shutdown()
            MyRobot.actuators.motors.set_speeds(0,0)
            print("stop the motors \n")
            return -1 

    #gère le control
    if(MyRobot.isON()):
        follow(scan_data,MyRobot)
        return 1