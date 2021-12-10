import numpy as np
#import RPi.GPIO as GPIO
#from robot import *
#from displacement import *
#from Talk2DE0 import *
import time
#pip install simple-pid
#from simple_pid import PID
import math
#import spidev
#https://pypi.org/project/simple-pid/#description

P_dw = 1;I_dw = 0;D_dw = 0;
P_s = 50;I_s = 0;D_s = 0;

P_d = 1;I_d = 0;D_d = 0;
P_a = 1;I_a = 0;D_a = 0;


deltat = 1e-3   #time btwn two mesures of the encoders
b = 0.2345      #lenght btwn wheels
D_odo = 0.045           #odometer_diameter
D_wheel = 0.06          #wheels diameter 

def limiter(val,MIN,MAX):
    val = max(val,MIN)
    val = min(val,MAX)
    return val

class PID(object):
    def __init__(self,P,I,D,MIN=None,MAX=None):
        self.P = P
        self.I = I
        self.D = D
        self.MIN = MIN
        self.MAX = MAX
        self.sp = 0
        #e_k -> see
        #https://jckantor.github.io/CBE30338/04.01-Implementing_PID_Control_with_Python_Yield_Statement.html
        self.e = np.array([0]) #can be reduced by juste knowing e_k ; e_k-1 and sum(e_k'*deltat)

    def limiter(val,MIN,MAX):
        val = max(val,MIN)
        val = min(val,MAX)
        return val

    def set_pid(self,P,I,D):
        self.P = P
        self.I = I
        self.D = D

    def set_limit(MIN = None,MAX = None):
        self.MIN = MIN
        self.MAX = MAX

    def set_setpoint(self,sp):
        self.sp = sp

    def compute(self,mes):
        P = self.P; I = self.I; D = self.D
        e_k = self.sp - mes
        self.e = np.append(self.e,e_k)
        output = P*e_k
        output += np.sum(self.e)*deltat*I
        output += (self.e[-1]-self.e[-2])/deltat*D
        output = limiter(output,self.MIN,self.MAX)
        return output

    def command(self,mes,verbose=0):
        output = self.compute(mes)
        if (verbose):print("ref {};mes {}::out {}".format(self.sp,mes,output))
        return output


class Controller(object):
    def __init__(self, MyRobot):
        self.thread_exit = 1
        self.MyRobot = MyRobot 
        #actual values
        self.theta = 0
        self.x = 0
        self.y = 0
        #mes
        self.mes_r = 0
        self.mes_l = 0
        self.d_mes_l = 0
        self.d_mes_r = 0
        #PID's controller for each wheel
        self.PID_dist_l = PID(P_dw,I_dw,D_dw,-50,50)
        self.PID_speed_l = PID(P_s,I_s,D_s,-100,100)
        self.PID_dist_r = PID(P_dw,I_dw,D_dw,-50,50)
        self.PID_speed_r = PID(P_s,I_s,D_s,-60,60)
        self.PID_dist = PID(P_d,I_d,D_d,-2,2)
        self.PID_angle = PID(P_a,I_a,D_a,-3,3)
        #DEO nano talk
        #self.DE02RPI = DE02Rpi(self)
        #self.DE02RPI.start_thread()

    def update_pos(self,x,y,theta):
        self.x = x
        self.y = y
        self.theta = theta

    def compute_update_pos(self,mes_l,mes_r,Encoder = 0):
        D = D_odo
        if (Encoder):
            D = D_wheel
        d_l = mes_l * D
        d_r = mes_r * D
        d = (d_r+d_l)/2
        phi = (d_r-d_l)/b
        x = self.x + d * math.cos(self.theta + phi/2)
        y = self.y + d * math.sin(self.theta + phi/2)
        theta = self.theta + phi
        self.update_pos(x,y,theta)

    def set_measures(self,mes_l,mes_r):
        self.mes_r = mes_r
        self.mes_l = mes_l
        self.d_mes_l += mes_l
        self.d_mes_r += mes_r

    def compute_all_chain(self,Encoder=0):
        D = D_odo
        if (Encoder): D = D_wheel
        #first
        dist_r = self.d_mes_r*2*math.pi*D/2
        dist_l = self.d_mes_l*2*math.pi*D/2
        omega_l_meas = self.d_mes_l*2*math.pi/deltat
        omega_r_meas = self.d_mes_r*2*math.pi/deltat

        theta_mes = (self.d_mes_r-self.d_mes_l)/b
        d_mes = (self.d_mes_l+self.d_mes_r)/2
        theta_p = self.PID_angle.output_value(theta_mes)
        d_p = self.PID_dist.output_value(d_mes)
        #second
        d_r = (theta_p*b+2*d_p)/2
        d_l = (2*d_p-theta_p*b)/2
        #wheels
        self.PID_dist_l.set_setpoint(d_l)
        omega_l = self.PID_dist_l.output_value(dist_l,1)
        self.PID_dist_r.set_setpoint(d_r)
        omega_r = self.PID_dist_r.output_value(dist_r)
        self.PID_speed_l.set_setpoint(omega_l)
        u_l = self.PID_speed_l.output_value(omega_l_meas,1)
        self.PID_speed_r.set_setpoint(omega_r)
        u_r = self.PID_speed_r.output_value(omega_r_meas)
        #send_to_motor
        return u_l,u_r
        #self.MyRobot.set_speeds(u_l,u_r)
