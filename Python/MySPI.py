import RPi.GPIO as GPIO
from time import sleep
import spidev
from simple_pid import PID
import math
import numpy as np
import matplotlib.pyplot as plt

from displacement import *


MySPI_FPGA = spidev.SpiDev()
MySPI_FPGA.open(0,1)
MySPI_FPGA.max_speed_hz = 500000

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)


def count(spi):
    treshold = 4192

    return spi[4] + (spi[3] << 8) + (spi[2] << 16) + (spi[1] << 24) - treshold


def Omega_mes_r(Enc):
    if (Enc):
        ToSPI_rightEnc = [0x01, 0x00, 0x00, 0x00, 0x00]
        countRightEnc = count(MySPI_FPGA.xfer2(ToSPI_rightEnc))
        return countRightEnc * (2 * math.pi / (2048 * 4))
    else :
        ToSPI_rightOdo = [0x03, 0x00, 0x00, 0x00, 0x00]
        countRightOdo = count(MySPI_FPGA.xfer2(ToSPI_rightOdo))
        return countRightOdo * (2 * math.pi / (2048 * 4))


def Omega_mes_l(Enc):
    if (Enc):
        ToSPI_leftEnc = [0x00, 0x00, 0x00, 0x00, 0x00]
        countLeftEnc = -1 * count(MySPI_FPGA.xfer2(ToSPI_leftEnc))
        return countLeftEnc * (2 * math.pi / (2048 * 4))
    else :
        ToSPI_leftOdo = [0x02, 0x00, 0x00, 0x00, 0x00]
        countLeftOdo = -1 * count(MySPI_FPGA.xfer2(ToSPI_leftOdo))
        return countLeftOdo * (2 * math.pi / (2048 * 4))


def controller(omega_ref_r, omega_ref_l, omega_mes_r, omega_mes_l):
    P = 10
    I = 0.5
    D = 0

    PID_r = PID(P, I, D, setpoint=omega_ref_r)
    PID_r.output_limits = (-100, 100)  # pourcentage in PWM
    PID_l = PID(P, I, D, setpoint=omega_ref_l)
    PID_l.output_limits = (-100, 100)  # pourcentage in PWM

    # commande en sortie du controller
    u_r = PID_r(omega_mes_r)
    u_l = PID_r(omega_mes_l)

    return [u_r, u_l]


def speed_wo_controller(DC, time):
    motors = Motors()
    motors.start_all()

    n_dt = np.arange(0, time, 0.001)
    omega_mes_r = []
    omega_mes_l = []

    motors.set_speeds(DC, DC)
    for i in n_dt:
        omega_mes_r.append(Omega_mes_r(1))
        omega_mes_l.append(Omega_mes_l(1))

    motors.stop_all()

    fig, (ax_r, ax_l) = plt.subplots(2, sharex=True)
    ax_r.plot(n_dt, omega_mes_r, label='omega_mes_r')
    ax_r.plot(n_dt, DC, label='DC')
    ax_r.legend()
    ax_r.set_title('omega_mes_r')
    ax_r.set(xlabel='omega [rad/s]')

    ax_l.plot(n_dt, omega_mes_l, label='omega_mes_l')
    ax_l.plot(n_dt, DC, label='DC')
    ax_l.legend()
    ax_l.set_title('omega_mes_l')
    ax_r.set(xlabel='omega [rad/s]', ylabel='time [s]')

    plt.show()

    return


def speed_w_controller():
    dt = 0.001
    end = 5/dt
    time = np.arange(0, 5, dt)
    omega_ref = np.zeros(end)
    ref1 = 2/dt
    ref2 = 4/dt
    omega_ref[0:ref1] = 2
    omega_ref[ref1:ref2] = 4
    omega_ref[ref2:end] = 0

    motors = Motors()
    motors.start_all()

    omega_mes_r = []
    omega_mes_l = []
    u_r_list = []
    u_l_list = []

    motors.set_speeds(omega_ref[0], omega_ref[0])
    for i in range(int(end)):

        omega_mes_r.append(Omega_mes_r(1))
        omega_mes_l.append(Omega_mes_l(1))

        u_r, u_l = controller(omega_ref[i], omega_ref[i], omega_mes_r[i], omega_mes_l[i])
        u_r_list.append(u_r)
        u_l_list.append(u_l)
        motors.set_speeds(u_l, u_r)

    motors.stop_all()

    fig, (ax_r, ax_l) = plt.subplots(2, sharex=True)
    ax_r.plot(time, omega_mes_r, label='omega_mes_r')
    ax_r.plot(time, omega_ref, label='omega_ref')
    ax_r.plot(time, u_r_list, label='u_r')
    ax_r.legend()
    ax_r.set_title('omega_mes_r')
    ax_r.set(xlabel='omega [rad/s]')

    ax_l.plot(time, omega_mes_l, label='omega_mes_l')
    ax_l.plot(time, omega_ref, label='omega_ref')
    ax_l.plot(time, u_l_list, label='u_l')
    ax_l.legend()
    ax_l.set_title('omega_mes_l')
    ax_r.set(xlabel='omega [rad/s]', ylabel='time [s]')

    plt.show()
    
    return











