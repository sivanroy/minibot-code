import RPi.GPIO as GPIO
import spidev
from simple_pid import PID
import math


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
        self.Ki = 0
        self.Kd = 0

        self.PID_r = PID(self.Kp, self.Ki, self.Kd, setpoint=self.w_ref_r)
        self.PID_r.output_limits = (-50, 50)
        self.PID_r.sample_time = 0.001
        self.PID_l = PID(self.Kp, self.Ki, self.Kd, setpoint=self.w_ref_l)
        self.PID_l.output_limits = (-50, 50)
        self.PID_l.sample_time = 0.001

    def set_w_ref(self, cmd_theta, cmd_dist):
        w_ref_r = cmd_dist - cmd_theta
        w_ref_l = cmd_dist + cmd_theta
        self.w_ref_r = w_ref_r
        self.w_ref_l = w_ref_l

        self.PID_r.setpoint = w_ref_r
        self.PID_l.setpoint = w_ref_l

        self.w_ref_r_list.append(w_ref_r)
        self.w_ref_l_list.append(w_ref_l)

    def command_speed(self):
        self.w_mes_r, self.w_mes_l = w_mes(self.MySPI_FPGA)
        self.w_mes_r_list.append(self.w_mes_r)
        self.w_mes_l_list.append(self.w_mes_l)
        self.print_w()

        cmd_r = self.PID_r(self.w_mes_r)
        cmd_l = self.PID_l(self.w_mes_l)
        print(cmd_r, '::', cmd_l)

        return cmd_r, cmd_l

    def return_w_list(self):
        return [self.w_ref_r_list, self.w_ref_l_list, self.w_mes_r_list, self.w_mes_l_list]

    def print_w(self):
        print(self.w_ref_r, '|', self.w_mes_r)
        print(self.w_ref_l, '|', self.w_mes_l)


class Position_controller(object):
    def __init__(self):
        self.theta = 0
        self.dist = 0
        self.theta_ref = 0
        self.dist_ref = 300

        self.Kp_theta = 1/30
        self.Ki_theta = 0
        self.Kd_theta = 0
        self.Kp_dist = 0.006
        self.Ki_dist = 0
        self.Kd_dist = 0

    def set_pos(self, dist, theta):
        self.theta = - (theta - 180)
        self.dist = dist

    def command_theta(self):
        d_theta = self.theta - self.theta_ref
        cmd_theta = d_theta * self.Kp_theta
        #print('cmd_theta =', cmd_theta)
        return cmd_theta

    def command_dist(self):
        d_dist = self.dist - self.dist_ref
        cmd_dist = d_dist * self.Kp_dist
        #print('cmd_dist', cmd_dist)
        return cmd_dist
