import numpy as np
import RPi.GPIO as GPIO
from robot import *
from displacement import *
import time
#pip install simple-pid
from simple_pid import PID
#https://pypi.org/project/simple-pid/#description

P = 2
I = 0.05
D = 0

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

    def output_value_l(self):
    	return self.PID_l(self.actual_value_l)

    def output_value_r(self):
    	return self.PID_r(self.actual_value_r)

class DEO2Rpi(object):
	def __init__(self):
		self.smthing = 0

	def dright(self):
		dright = 0
		return dright

	def dleft(self):
		dleft = 0
		return dleft

class Controller(object):
    def __init__(self):
    	#actual values
        self.theta = 0
        self.x = 0
        self.y = 0
        #PID's controller for each wheel
        self.PID_obj = PID_obj()
        #DEO nano talk
        self.DE02RPI = DEO2Rpi()
        #values of setpoint
        self.omega_ref_l = 0
        self.omega_ref_r = 0
        #setpoint in x,y domain
        self.x_ref = 0
        self.y_ref = 0

    def send_to_motors(self,motors):
    	speedl = self.PID_obj.output_value_l()
    	speedr = self.PID_obj.output_value_r()
    	motors.set_speeds(speedl, speedr)

    def set_cartesian_ref(self,new_x,new_y):
    	self.x_ref = new_x
    	self.y_ref = new_y

        sp_r = 0
        sp_l = 0

        self.PID_obj.set_setpoint_r(sp_r)
        self.PID_obj.set_setpoint_l(sp_l) 
