import numpy as np
import RPi.GPIO as GPIO
from robot import *
from displacement import *
import time
#pip install simple-pid
from simple_pid import PID
import math
import spidev
#https://pypi.org/project/simple-pid/#description

P = 2
I = 0.05
D = 0

deltat = 1e-3 #time btwn two mesures of the encoders
b = 0.2345 #lenght btwn wheels

class PID_obj(object):
	def __init__(self):
        self.PID_l = PID(P,I,D,setpoint = 0)
        PID_l.output_limits = (-100,100) #pourcentage in PWM
        self.PID_r = PID(P,I,D,setpoint = 0)
        PID_r.output_limits = (-100,100) #pourcentage in PWM
        self.omega_mes_l = 0 #omega_l ?
        self.omega_mes_r = 0 #omega_r ?

    def set_setpoint_l(self,sp):
    	self.PID_l.setpoint = sp

    def set_setpoint_r(self,sp):
    	self.PID_l.setpoint = sp

    def set_mes(self,omega_mes_l,omega_mes_r):
        self.omega_mes_l = omega_mes_l
        self.omega_mes_r = omega_mes_r

    def output_value_l(self):
    	return self.PID_l(self.omega_mes_l)

    def output_value_r(self):
    	return self.PID_r(self.omega_mes_r)


class Controller(object):
    def __init__(self, MyRobot):
        self.thread_exit = 0
        self.MyRobot = MyRobot 
    	#actual values
        self.theta = 0
        self.x = 0
        self.y = 0
        #PID's controller for each wheel
        self.PID_obj = PID_obj()
        #DEO nano talk
        self.DE02RPI = DEO2Rpi(self)
        #values of setpoint
        #self.omega_ref_l = 0
        #self.omega_ref_r = 0
        #setpoint in x,y domain
        self.x_ref = 0
        self.y_ref = 0

    def update_pos(self,x,y,theta):
        self.x = x
        self.y = y
        self.theta = theta

    def compute_pos(self,omega_mes_l,omega_mes_r):
        d_l = omega_mes_l*deltat
        d_r = omega_mes_r*deltat
        d = (d_r+d_l)/2
        phi = (d_r-d_l)/b
        x = self.x + d*math.cos(self.theta+phi/2)
        y = self.y + d*math.sin(self.theta+phi/2)
        theta = self.theta+phi
        self.update_pos(x,y,theta)

    def send_to_motors(self):
    	speedl = self.PID_obj.output_value_l()
    	speedr = self.PID_obj.output_value_r()
    	MyRobot.set_speeds(speedl, speedr)

    def set_cartesian_ref(self,new_x,new_y):
    	self.x_ref = new_x
    	self.y_ref = new_y

        #comput d_r and d_l from
        # d =   (d_r+d_l)/2
        # phi = (d_r-d_l)/b
        deltax = self.x_ref-self.x
        deltay = self.y_ref-self.y

        d = sqrt(deltax**2+deltay**2)
        alpha = math.asin(deltay/d)

        phi = self.theta-alpha

        d_r = (b*phi+2*d)/2
        d_l = (2*d-b*phi)/2

        sp_r = d_r/deltat
        sp_l = d_l/deltat

        self.PID_obj.set_setpoint_r(sp_r)
        self.PID_obj.set_setpoint_l(sp_l) 
