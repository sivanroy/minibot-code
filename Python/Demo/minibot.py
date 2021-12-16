import numpy as np
import matplotlib.pyplot as plt
from motors import *
from controllers import *
from buttons import *


MINDIST = 300
MAXDIST = 600
ERROR = 10

def count(spi):
    treshold = 4192
    return spi[4] + (spi[3] << 8) + (spi[2] << 16) + (spi[1] << 24) - treshold


def closer(scan_data):
    dist = scan_data[0]
    theta = scan_data[1]
    max_index = np.where(dist == np.amin(np.array(dist)))[0][0]
    dist_p = dist[max_index]
    theta_p = theta[max_index]
    return dist_p, theta_p


class Infos(object):
    def __init__(self):
        self.direction = 0
        self.speed = 0
        self.position = [0,0]
    def print_infos(self):
        print("------------------------------\n\
            button1 :   {d}\n\
            direction : {d}\n\
            speed :     {d}\n\
            position : [{:2f},{:2f}]\n\
            ------------------------------\n".format(\
                self.button1,self.direction,self.speed,\
                self.position[0],self.position[1]))


class Minibot(object):
    def __init__(self):
        self.ON = 0
        self.motors = Motors()
        self.motors.start_all()
        self.s_controller = Speed_controller()
        self.p_controller = Position_controller()
        self.buttons = Buttons()
        self.infos = Infos()
        self.count_r = 0
        self.count_l = 0

    def follow(self, scan_data):
        dist, theta = closer(scan_data)
        print(dist, theta)

        self.p_controller.set_pos(dist, theta)
        cmd_theta = self.p_controller.command_theta()
        cmd_dist = self.p_controller.command_dist()

        self.s_controller.set_w_ref(cmd_theta, cmd_dist)
        cmd_r, cmd_l = self.s_controller.command_speed()

        self.motors.set_speeds(cmd_l, cmd_r)

    def calib(self, MySPI_FPGA):
        ToSPI_rightEnc = [0x01, 0x00, 0x00, 0x00, 0x00]
        countRightEnc = count(MySPI_FPGA.xfer2(ToSPI_rightEnc))

        ToSPI_leftEnc = [0x00, 0x00, 0x00, 0x00, 0x00]
        countLeftEnc = -count(MySPI_FPGA.xfer2(ToSPI_leftEnc))

        print(countRightEnc, countLeftEnc)
        self.count_r += countRightEnc
        self.count_l += countLeftEnc

    def return_count(self):
        return self.count_r, self.count_l

    def print_infos(self):
        self.infos.print_infos()

    def isON(self):
        return self.ON

    def activate(self):
        print("Activate the minibot \n")
        self.ON = 1

    def shutdown(self):
        print("Shutdown the minibot \n")
        self.motors.stop_all()
        self.ON = 0

    def plot_w(self):
        w_list = self.s_controller.return_w_list()
        w_ref_r = w_list[0]
        w_ref_l = w_list[1]
        w_mes_r = w_list[2]
        w_mes_l = w_list[3]

        t = np.linspace(0, len(w_ref_r), len(w_ref_r))

        fig, (ax_r, ax_l) = plt.subplots(2, sharex=True)
        ax_r.plot(t, w_mes_r, label='w_mes_r')
        ax_r.plot(t, w_ref_r, label='w_ref_r')
        ax_r.legend()
        ax_r.set_title('Speeds plot')
        ax_r.set(ylabel='w [rad/s]')

        ax_l.plot(t, w_mes_l, label='w_mes_l')
        ax_l.plot(t, w_ref_l, label='w_ref_l')
        ax_l.legend()
        ax_l.set(ylabel='w [rad/s]', xlabel='cnt')

        plt.show()