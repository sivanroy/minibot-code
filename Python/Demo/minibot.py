import numpy as np
import matplotlib.pyplot as plt
from motors import *
from controller import *
from buttons import *


MINDIST = 300
MAXDIST = 600
ERROR = 10
SPEED_AVRG = 2
SPEED_MAX = 6

SPEED_FACTOR = (SPEED_MAX - SPEED_AVRG)/MAXDIST


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
        self.controller = Controller()
        self.buttons = Buttons()
        self.infos = Infos()


    def follow(self, scan_data):
        dist, theta = closer(scan_data)

        if dist < MINDIST or dist > MAXDIST:
            self.motors.stop_all()

        ########## front left ##########
        elif 90 < theta < 180 - ERROR:
            d_theta = -(theta - 180)
            dbs = dist - MINDIST  # dbs = distance before stop
            w_ref_r = SPEED_FACTOR * dbs + SPEED_AVRG
            w_ref_l = w_ref_r * ((90 + ERROR - 2*d_theta)/(90 - ERROR))

            self.controller.set_w_ref(w_ref_r, w_ref_l)
            cmd_r, cmd_l = self.controller.command()
            self.motors.set_speeds(cmd_l, cmd_r)

        ########## front right ##########
        elif 180 + ERROR < theta < 270:
            d_theta = theta - 180
            dbs = dist - MINDIST
            w_ref_l = SPEED_FACTOR * dbs + SPEED_AVRG
            w_ref_r = w_ref_l * ((90 + ERROR - 2*d_theta)/(90 - ERROR))

            self.controller.set_w_ref(w_ref_r, w_ref_l)
            cmd_r, cmd_l = self.controller.command()
            self.motors.set_speeds(cmd_l, cmd_r)

        ########## back left ##########
        elif 0 < theta < 90:
            w_ref_r = SPEED_MAX
            w_ref_l = -w_ref_r

            self.controller.set_w_ref(w_ref_r, w_ref_l)
            cmd_r, cmd_l = self.controller.command()
            self.motors.set_speeds(cmd_l, cmd_r)

        ########## back right ##########
        elif 270 < theta < 360:
            w_ref_l = SPEED_MAX
            w_ref_r = -w_ref_l

            self.controller.set_w_ref(w_ref_r, w_ref_l)
            cmd_r, cmd_l = self.controller.command()
            self.motors.set_speeds(cmd_l, cmd_r)

        ########## in front of ##########
        else:
            dbs = dist - MINDIST
            w_ref_r = SPEED_FACTOR * dbs + SPEED_AVRG
            w_ref_l = w_ref_r

            self.controller.set_w_ref(w_ref_r, w_ref_l)
            cmd_r, cmd_l = self.controller.command()
            self.motors.set_speeds(cmd_l, cmd_r)

        return

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
        w_list = self.controller.return_w_list()
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

