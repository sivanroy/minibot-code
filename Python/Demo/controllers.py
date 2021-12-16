import RPi.GPIO as GPIO
import spidev
from simple_pid import PID
import math
import numpy as np

d_ref = 300
deltat = 2e-3

def DE02RPi_init():
    MySPI_FPGA = spidev.SpiDev()
    MySPI_FPGA.open(0,1)
    MySPI_FPGA.max_speed_hz = 500000

    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    return


def count(spi):
    treshold = 4192
    return spi[4] + (spi[3] << 8) + (spi[2] << 16) + (spi[1] << 24) - treshold


def w_mes(MySPI_FPGA):
    ToSPI_rightEnc = [0x01, 0x00, 0x00, 0x00, 0x00]
    countRightEnc = count(MySPI_FPGA.xfer2(ToSPI_rightEnc))
    w_mes_r = countRightEnc * 1000 * (2 * math.pi / (2048 * 4))

    ToSPI_leftEnc = [0x00, 0x00, 0x00, 0x00, 0x00]
    countLeftEnc = -1 * count(MySPI_FPGA.xfer2(ToSPI_leftEnc))
    w_mes_l = countLeftEnc * 1000 * (2 * math.pi / (2048 * 4))

    return w_mes_r, w_mes_l


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

    def limiter(self, val, MIN, MAX):
        if (MIN != None):
            val = max(val,MIN)
        if (MAX != None):
            val = min(val,MAX)
        return val

    def set_pid(self, P, I, D):
        self.P = P
        self.I = I
        self.D = D

    def set_limit(self, MIN = None, MAX = None):
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

    def command(self, mes, verbose=0):
        output = self.compute(mes)
        if (verbose):
            print("ref {};mes {}::out {}".format(self.sp,mes,output))
        return output


class Speed_controller(object):
    def __init__(self):
        self.MySPI_FPGA = spidev.SpiDev()
        self.MySPI_FPGA.open(0, 1)
        self.MySPI_FPGA.max_speed_hz = 500000
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)

        self.w_ref_r = 0
        self.w_ref_l = 0
        self.w_mes_r = 0
        self.w_mes_l = 0
        self.w_ref_r_list = [0]
        self.w_ref_l_list = [0]
        self.w_mes_r_list = [0]
        self.w_mes_l_list = [0]
        self.Kp = 60
        self.Ki = 0.6
        self.PID_r = PID(self.Kp, self.Ki, 0, -80, 80)
        self.PID_l = PID(self.Kp, self.Ki, 0, -80, 80)

    def set_w_ref(self, cmd_theta, cmd_dist):
        w_ref_r = cmd_dist - cmd_theta
        w_ref_l = cmd_dist + cmd_theta
        self.PID_r.set_setpoint(w_ref_r)
        self.PID_l.set_setpoint(w_ref_l)

        self.w_ref_r = w_ref_r
        self.w_ref_l = w_ref_l
        #self.w_ref_r_list.append(w_ref_r)
        #self.w_ref_l_list.append(w_ref_l)

    def command_speed(self):
        self.w_mes_r, self.w_mes_l = w_mes(self.MySPI_FPGA)
        #self.w_mes_r_list.append(self.w_mes_r)
        #self.w_mes_l_list.append(self.w_mes_l)
        self.print_w()

        # commande en sortie du controller
        u_r = self.PID_r.command(self.w_ref_r)
        u_l = self.PID_r.command(self.w_ref_l)
        print(u_r, '::', u_l)

        return u_r, u_l

    def return_w_list(self):
        return [self.w_ref_r_list, self.w_ref_l_list, self.w_mes_r_list, self.w_mes_l_list]

    def print_w(self):
        print(self.w_ref_r, '|', self.w_mes_r)
        print(self.w_ref_l, '|', self.w_mes_l)


class Position_controller(object):
    def __init__(self):
        self.theta = 0
        self.dist = 0
        self.Kp_theta = (1/30)
        self.Ki_theta = 0
        self.Kd_theta = 0
        self.Kp_dist = 0.006
        self.Ki_dist = 0
        self.Kd_dist = 0

    def set_pos(self, dist, theta):
        self.theta = - (theta - 180) * 2*np.pi/360
        self.dist = dist/1000

    def command_theta(self):
        cmd_theta = self.theta * self.Kp_theta
        return cmd_theta

    def command_dist(self):
        cmd_dist = (self.dist - 0.3) * self.Kp_dist
        return cmd_dist
