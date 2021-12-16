import RPi.GPIO as GPIO
from time import sleep
import time
import spidev
from simple_pid import PID
import math
import numpy as np
import matplotlib.pyplot as plt

from motors import *


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
    P = 100
    I = 1
    D = 0

    PID_r = PID(P, I, D, setpoint=omega_ref_r)
    PID_r.output_limits = (-50, 50)  # pourcentage in PWM
    PID_l = PID(P, I, D, setpoint=omega_ref_l)
    PID_l.output_limits = (-50, 50)  # pourcentage in PWM

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


def speed_controller(end, w_ref1, w_ref2, P, I, D):
    t1 = int(end * 1/10)
    t2 = end * 5/10
    t3 = end * 8/10
    dt_means = []
    t_actual = 0
    t = []

    w_mes_r = []
    w_mes_l = []
    w_ref = 0
    w_ref_list = []
    w_ref_list_moy = []

    w_mes_r_moy = []
    w_mes_l_moy = []
    w_ref_moy = []

    PID_r = PID(P, I, D, setpoint=w_ref)
    PID_r.output_limits = (-50, 50)
    PID_r.sample_time = 0.001
    PID_l = PID(P, I, D, setpoint=w_ref)
    PID_l.output_limits = (-50, 50)
    PID_l.sample_time = 0.001

    motors = Motors()
    motors.start_all()

    c = 0
    nbr_moy = 10
    for i in range(end):
        t.append(t_actual)
        start = time.time()

        if i < t1:
            w_ref = 0
        elif t1 <= i < t2:
            w_ref = w_ref1
        elif t2 <= i < t3:
            w_ref = w_ref2
        else:
            w_ref = 0

        w_ref_list.append(w_ref)
        PID_r.setpoint = w_ref
        PID_l.setpoint = w_ref

        w_mes_r.append(Omega_mes_r(1))
        w_mes_l.append(Omega_mes_l(1))

        u_r = PID_r(w_mes_r[i])
        u_l = PID_l(w_mes_l[i])

        motors.set_speeds(u_l, u_r)

        if c == nbr_moy-1:

            sum_r = 0
            sum_l = 0
            for j in range(i-nbr_moy, i):
                sum_r += w_mes_r[j]
                sum_l += w_mes_l[j]
            w_mes_r_moy.append(sum_r/nbr_moy)
            w_mes_l_moy.append(sum_l/nbr_moy)
            w_ref_list_moy.append(w_ref)
            w_ref_moy.append(w_ref)
            c = 0
        else:
            c += 1
        #sleep(0.0006)
        stop = time.time()
        dt_means.append(stop - start)
        t_actual += (stop - start)

    motors.stop_all()

    sum_t = 0
    for t in dt_means:
        sum_t += t
    dt_mean = sum_t/len(dt_means)
    print('dt mean =', dt_mean)

    fig, (ax_r, ax_l) = plt.subplots(2, sharex=True)
    ax_r.plot(w_mes_r_moy, label='omega_mes_r')
    ax_r.plot(w_ref_list_moy, label='omega_ref')
    ax_r.legend()
    ax_r.set_title('omega_mes_r')
    ax_r.set(ylabel='omega [rad/s]')

    ax_l.plot(w_mes_l_moy, label='omega_mes_l')
    ax_l.plot(w_ref_list_moy, label='omega_ref')
    ax_l.legend()
    ax_l.set(ylabel='omega [rad/s]')

    plt.show()

    return


def speed_controller_p(end, w_ref1, w_ref2, P, I, D):
    t1 = int(end * 1/10)
    t2 = end * 5/10
    t3 = end * 8/10
    dt_means = []
    t_actual = 0
    t = []

    nbr_moy = 10
    w_mes_r = []
    w_mes_l = []
    w_ref = 0
    w_ref_list = np.linspace(w_ref1, w_ref2, end)
    w_ref_list_moy = np.linspace(w_ref1, w_ref2, end/nbr_moy)

    w_mes_r_moy = []
    w_mes_l_moy = []
    w_ref_moy = []

    PID_r = PID(P, I, D, setpoint=w_ref1)
    PID_r.output_limits = (-50, 50)
    PID_r.sample_time = 0.001
    PID_l = PID(P, I, D, setpoint=w_ref2)
    PID_l.output_limits = (-50, 50)
    PID_l.sample_time = 0.001

    motors = Motors()
    motors.start_all()

    c = 0

    for i in range(end):
        t.append(t_actual)
        start = time.time()

        w_ref = w_ref_list[i]

        PID_r.setpoint = w_ref
        PID_l.setpoint = w_ref

        w_mes_r.append(Omega_mes_r(1))
        w_mes_l.append(Omega_mes_l(1))

        u_r = PID_r(w_mes_r[i])
        u_l = PID_l(w_mes_l[i])

        motors.set_speeds(u_l, u_r)

        if c == nbr_moy-1:

            sum_r = 0
            sum_l = 0
            for j in range(i-nbr_moy, i):
                sum_r += w_mes_r[j]
                sum_l += w_mes_l[j]
            w_mes_r_moy.append(sum_r/nbr_moy)
            w_mes_l_moy.append(sum_l/nbr_moy)
            #w_ref_list_moy.append(w_ref)
            w_ref_moy.append(w_ref)
            c = 0
        else:
            c += 1
        #sleep(0.0006)
        stop = time.time()
        dt_means.append(stop - start)
        t_actual += (stop - start)

    motors.stop_all()

    sum_t = 0
    for t in dt_means:
        sum_t += t
    dt_mean = sum_t/len(dt_means)
    print('dt mean =', dt_mean)

    fig, (ax_r, ax_l) = plt.subplots(2, sharex=True)
    ax_r.plot(w_mes_r_moy, label='omega_mes_r')
    ax_r.plot(w_ref_list_moy, label='omega_ref')
    ax_r.legend()
    ax_r.set_title('omega_mes_r')
    ax_r.set(ylabel='omega [rad/s]')

    ax_l.plot(w_mes_l_moy, label='omega_mes_l')
    ax_l.plot(w_ref_list_moy, label='omega_ref')
    ax_l.legend()
    ax_l.set(ylabel='omega [rad/s]')

    plt.show()

    return


speed_controller(10000, 4, 2, 60, 0, 0)
#speed_controller_p(5000, 0, 4, 60, 0, 0)




