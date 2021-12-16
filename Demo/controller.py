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
#P_s = 402.6;I_s = P_s/1.028;D_s = 0; #wheel speed controller param
#P_d = 114.13  ;I_d = 64.9  ;D_d = 4.47;
#P_a = 13.88  ;I_a = 24.9  ;D_a = 3.11;
#P_d = 10  ;I_d = 0  ;D_d = 0;
#P_a = 10  ;I_a = 0  ;D_a = 0;

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
        #self.e = np.array([0]) #can be reduced by juste knowing e_k ; e_k-1 and sum(e_k'*deltat)
        self.sum = 0
        self.last = 0
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
        e_km1 = self.last
        self.sum += e_k
        #self.e = np.append(self.e,e_k)
        output_P = P*e_k
        output_I = self.sum*deltat*I
        output_D = (e_k-e_km1)/deltat*D
        if (self.doPlot):
            self.P_a.append(output_P);self.I_a.append(output_I);self.D_a.append(output_D);
        output = self.limiter(output_P+output_I+output_D,self.MIN,self.MAX)
        self.last = e_k 
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
        #DEO nano talk
        self.DE02RPI = DE02Rpi(self)