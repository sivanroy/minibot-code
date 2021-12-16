import numpy as np
import RPi.GPIO as GPIO
from robot import *
from displacement import *
from Talk2DE0 import *
import time
#pip install simple-pid
#from simple_pid import PID
import math
import spidev
import matplotlib.pyplot as plt
#https://pypi.org/project/simple-pid/#description

#P_dw = 1  ;I_dw = 0  ;D_dw = 0;

P_s = 402.6/100;I_s = 4; D_s=0#.005#P_s/1.028;D_s = 0; #wheel speed controller param


deltat = 2e-3   #time btwn two mesures of the encoders
b = 0.2345      #lenght btwn wheels
D_odo = 0.045           #odometer_diameter
D_wheel = 0.06          #wheels diameter 

def limiter(val,MIN,MAX):
    val = max(val,MIN)
    val = min(val,MAX)
    return val

class PID(object):
    def __init__(self,P,I,D,MIN=None,MAX=None,doPlot=0):
        self.doPlot = 0
        self.P = P
        self.I = I
        self.D = D
        self.MIN = MIN
        self.MAX = MAX
        self.sp = 0
        #e_k -> see
        #https://jckantor.github.io/CBE30338/04.01-Implementing_PID_Control_with_Python_Yield_Statement.html
        self.e = np.array([0]) #can be reduced by juste knowing e_k ; e_k-1 and sum(e_k'*deltat)
        self.P_a = []
        self.I_a = []
        self.D_a = []

    def limiter(self,val,MIN,MAX):
        if (MIN != None):
            val = max(val,MIN)
        if (MAX != None):
            val = min(val,MAX)
        return val

    def set_pid(self,P,I,D):
        self.P = P
        self.I = I
        self.D = D

    def set_limit(self,MIN = None,MAX = None):
        self.MIN = MIN
        self.MAX = MAX

    def set_setpoint(self,sp):
        self.sp = sp

    def compute(self,mes):
        P = self.P; I = self.I; D = self.D
        e_k = self.sp - mes
        self.e = np.append(self.e,e_k)
        output_P = P*e_k
        output_I = np.sum(self.e)*deltat*I
        output_D = (self.e[-1]-self.e[-2])/deltat*D
        if (self.doPlot):
            self.P_a.append(output_P);self.I_a.append(output_I);self.D_a.append(output_D);
        output = limiter(output_P+output_I+output_D,self.MIN,self.MAX)
        return output

    def command(self,mes,verbose=0):
        output = self.compute(mes)
        if (verbose):
            print("ref {};mes {}::out {}".format(self.sp,mes,output))
        return output


class Controller(object):
    def __init__(self, MyRobot):
        self.thread_exit = 1
        self.MyRobot = MyRobot 
        #actual values
        self.theta = 0
        self.x = 0
        self.y = 0
        self.x_track = [0]
        self.y_track = [0]
        self.theta_track = [0]

        #mes
        self.mes_r = 0
        self.mes_l = 0
        self.d_mes_l = 0
        self.d_mes_r = 0
        #PID's controller for each wheel
        self.PID_speed_l = PID(P_s,I_s,D_s,-30,60)
        self.PID_speed_r = PID(P_s,I_s,D_s,-30,60)

        #DEO nano talk
        self.DE02RPI = DE02Rpi(self)
        #self.DE02RPI.start_thread()

    def send_to_motors(self,u_l,u_r):
        self.MyRobot.set_speeds(u_l,u_r)

    def update_pos(self,x,y,theta):
        self.x = x
        self.y = y
        self.theta = theta

        self.x_track.append(self.x)
        self.y_track.append(self.y)
        self.theta_track.append(self.theta)

    def compute_update_pos(self,mes_l,mes_r,Encoder = 0):
        D = D_odo
        if (Encoder):
            D = D_wheel
        d_l = (mes_l * D/2) * deltat
        d_r = (mes_r * D/2) * deltat
        d = (d_r+d_l)/2
        phi = (d_r-d_l)/b
        x_new = self.x + d * math.cos(self.theta + phi/2)
        y_new = self.y + d * math.sin(self.theta + phi/2)
        theta_new = self.theta + phi
        self.update_pos(x_new,y_new,theta_new)

    def return_pos_track(self):
        return self.x_track, self.y_track

    def set_measures(self, mes_l, mes_r):
        self.mes_r = mes_r
        self.mes_l = mes_l
        self.d_mes_l = mes_l#+= mes_l
        self.d_mes_r = mes_r#+= mes_r

