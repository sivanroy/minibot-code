import RPi.GPIO as GPIO
from displacement import *
from buttons import *
class Sensors(object):
    def __init__(self):
        self.buttons = Buttons();
        self.lidar = None


class Actuators(object):
    def __init__(self):
        self.motors = Motors()
        self.motors.start_all()

    def set_speeds(self,speedl,speedr):
        self.motors.set_speeds(speedl,speedr)


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


class Robot(object):
    def __init__(self):
        self.ON = 0
        self.sensors = Sensors()
        self.actuators = Actuators()
        self.infos = Infos()

    def set_speeds(self, speedl,speedr):
        self.actuators.motors.set_speeds(speedl,speedr);

    def print_infos(self):
        self.infos.print_infos()

    def isON(self):
        return self.ON

    def activate(self):
        self.ON = 1

    def shutdown(self):
        self.ON = 0