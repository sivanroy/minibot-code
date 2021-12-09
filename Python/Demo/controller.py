import RPi.GPIO as GPIO
import spidev
from simple_pid import PID
import math


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


def w_mes():
    ToSPI_rightEnc = [0x01, 0x00, 0x00, 0x00, 0x00]
    countRightEnc = count(MySPI_FPGA.xfer2(ToSPI_rightEnc))
    w_mes_r = countRightEnc * 1000 * (2 * math.pi / (2048 * 4))

    ToSPI_leftEnc = [0x00, 0x00, 0x00, 0x00, 0x00]
    countLeftEnc = -1 * count(MySPI_FPGA.xfer2(ToSPI_leftEnc))
    w_mes_l = countLeftEnc * 1000 * (2 * math.pi / (2048 * 4))

    return w_mes_r, w_mes_l


class Controller(object):
    def __init__(self):
        DE02RPi_init()
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
        self.PID_r = PID(self.Kp, self.Ki, 0, setpoint=self.w_ref_r)
        self.PID_r.output_limits = (-100, 100)  # pourcentage in PWM
        self.PID_l = PID(self.Kp, self.Ki, 0, setpoint=self.w_ref_l)
        self.PID_l.output_limits = (-100, 100)  # pourcentage in PWM

    def set_w_ref(self, w_ref_r, w_ref_l):
        self.w_ref_r = w_ref_r
        self.w_ref_l = w_ref_l
        self.w_ref_r_list.append(w_ref_r)
        self.w_ref_l_list.append(w_ref_l)
        self.PID_r.setpoint(w_ref_r)
        self.PID_l.setpoint(w_ref_l)

    def command(self):
        self.w_mes_r, self.w_mes_l = w_mes()
        self.w_mes_r_list.append(self.w_mes_r)
        self.w_mes_l_list.append(self.w_mes_l)

        # commande en sortie du controller
        u_r = self.PID_r(self.w_mes_r)
        u_l = self.PID_r(self.w_mes_l)

        return [u_r, u_l]

    def return_w_list(self):
        return [self.w_ref_r_list, self.w_ref_l_list, self.w_mes_r_list, self.w_mes_l_list]
