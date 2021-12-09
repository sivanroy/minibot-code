import RPi.GPIO as GPIO
from time import sleep
import time
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
        return countRightEnc * 1000 * (2 * math.pi / (2048 * 4))
    else :
        ToSPI_rightOdo = [0x03, 0x00, 0x00, 0x00, 0x00]
        countRightOdo = count(MySPI_FPGA.xfer2(ToSPI_rightOdo))
        return countRightOdo * 1000 * (2 * math.pi / (2048 * 4))


def Omega_mes_l(Enc):
    if (Enc):
        ToSPI_leftEnc = [0x00, 0x00, 0x00, 0x00, 0x00]
        countLeftEnc = -1 * count(MySPI_FPGA.xfer2(ToSPI_leftEnc))
        return countLeftEnc * 1000 * (2 * math.pi / (2048 * 4))
    else :
        ToSPI_leftOdo = [0x02, 0x00, 0x00, 0x00, 0x00]
        countLeftOdo = -1 * count(MySPI_FPGA.xfer2(ToSPI_leftOdo))
        return countLeftOdo * 1000 * (2 * math.pi / (2048 * 4))


def controller(omega_ref_r, omega_ref_l, omega_mes_r, omega_mes_l):
    P = 60
    I = 0.6
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

    DClist = []
    n_dt = np.arange(0, time, 0.001)
    omega_mes_r = []
    omega_mes_l = []

    motors.set_speeds(DC, DC)
    for i in n_dt:
        DClist.append(DC)
        omega_mes_r.append(Omega_mes_r(1))
        omega_mes_l.append(Omega_mes_l(1))
        sleep(0.0005)

    motors.stop_all()

    fig, (ax_r, ax_l) = plt.subplots(2, sharex=True)
    ax_r.plot(n_dt, omega_mes_r, label='omega_mes_r')
    ax_r.plot(n_dt, DClist, label='DC')
    ax_r.legend()
    ax_r.set_title('omega_mes')
    ax_r.set(ylabel='omega [rad/s]')

    ax_l.plot(n_dt, omega_mes_l, label='omega_mes_l')
    ax_l.plot(n_dt, DClist, label='DC')
    ax_l.legend()
    ax_l.set(ylabel='omega [rad/s]', xlabel='time [s]')

    plt.show()

    return


def speed_w_controller():
    dt = 0.001
    end = int(9/dt)
    t = []
    omega_ref = np.zeros(end)
    ref1 = int(0.5/dt)
    ref2 = int(2.5/dt)
    ref3 = int(4.5/dt)
    ref4 = int(6.5/dt)
    ref5 = int(8.5/dt)
    omega_ref[0:ref1] = 0
    omega_ref[ref1:ref2] = 2
    omega_ref[ref2:ref3] = 4
    omega_ref[ref3:ref4] = -2
    omega_ref[ref4:ref5] = -4
    omega_ref[ref5:end] = 0

    motors = Motors()
    motors.start_all()

    omega_mes_r = []
    omega_mes_l = []
    u_r_list = []
    u_l_list = []

    t_actual = 0
    #motors.set_speeds(omega_ref[0], omega_ref[0])
    for i in range(int(end)):
        start = time.time()

        omega_mes_r.append(Omega_mes_r(1))
        omega_mes_l.append(Omega_mes_l(1))

        u_r, u_l = controller(omega_ref[i], omega_ref[i], omega_mes_r[i], omega_mes_l[i])
        u_r_list.append(u_r)
        u_l_list.append(u_l)
        motors.set_speeds(u_l, u_r)
        sleep(0.0005)

        end = time.time()
        t.append(end-start + t_actual)
        t_actual = t[i]

    motors.stop_all()

    w_mes_r_moy = []

    fig, (ax_r, ax_l) = plt.subplots(2, sharex=True)
    ax_r.plot(t, omega_mes_r, label='omega_mes_r')
    ax_r.plot(t, omega_ref, label='omega_ref')
    #ax_r.plot(t, u_r_list, label='u_r')
    ax_r.legend()
    ax_r.set_title('omega_mes_r')
    ax_r.set(ylabel='omega [rad/s]')

    ax_l.plot(t, omega_mes_l, label='omega_mes_l')
    ax_l.plot(t, omega_ref, label='omega_ref')
    #ax_l.plot(t, u_l_list, label='u_l')
    ax_l.legend()
    ax_l.set(ylabel='omega [rad/s]', xlabel='time [s]')

    plt.show()

    return


def speed_w_controller_ground():
    dt = 0.001
    end = int(3/dt)
    t = []
    omega_ref = np.zeros(end)
    ref1 = int(0.5/dt)
    ref2 = int(1.5/dt)
    ref3 = int(2.5/dt)

    omega_ref[0:ref1] = 0
    omega_ref[ref1:ref2] = 2
    omega_ref[ref2:end] = 3
    #omega_ref[ref3:ref4] = -2
    #omega_ref[ref4:ref5] = -4
    #omega_ref[ref5:end] = 0

    motors = Motors()
    motors.start_all()

    omega_mes_r = []
    omega_mes_l = []
    u_r_list = []
    u_l_list = []

    t_actual = 0
    #motors.set_speeds(omega_ref[0], omega_ref[0])
    for i in range(int(end)):
        start = time.time()

        omega_mes_r.append(Omega_mes_r(1))
        omega_mes_l.append(Omega_mes_l(1))

        u_r, u_l = controller(omega_ref[i], omega_ref[i], omega_mes_r[i], omega_mes_l[i])
        u_r_list.append(u_r)
        u_l_list.append(u_l)
        motors.set_speeds(u_l, u_r)
        sleep(0.0005)

        end = time.time()
        t.append(end-start + t_actual)
        t_actual = t[i]

    motors.stop_all()

    w_mes_r_moy = []

    fig, (ax_r, ax_l) = plt.subplots(2, sharex=True)
    ax_r.plot(t, omega_mes_r, label='omega_mes_r')
    ax_r.plot(t, omega_ref, label='omega_ref')
    #ax_r.plot(t, u_r_list, label='u_r')
    ax_r.legend()
    ax_r.set_title('omega_mes_r')
    ax_r.set(ylabel='omega [rad/s]')

    ax_l.plot(t, omega_mes_l, label='omega_mes_l')
    ax_l.plot(t, omega_ref, label='omega_ref')
    #ax_l.plot(t, u_l_list, label='u_l')
    ax_l.legend()
    ax_l.set(ylabel='omega [rad/s]', xlabel='time [s]')

    plt.show()

    return



#speed_wo_controller(2, 2)
#speed_w_controller()
speed_w_controller_ground()






