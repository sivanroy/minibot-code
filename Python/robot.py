import RPi.GPIO as GPIO
from displacement import motors, MAX_SPEED

class Sensors(object):
    def __init__(self):
        self.lidar = None

class Actuators(object):
    def __init__(self):
        self.motors = Motors()

    def set_speeds(self,speedl,speedr):
        self.motors.set_speeds(speedl,speedr)

class Infos(object):
    def __init__(self):
        self.button1 = 0
        self.direction = 0
        self.speed = 0
        self.position = [0,0]    
    def print_infos(self):
        print("------------------------------\n \
            button1 :   {d}\n\
            direction : {d}\n\
            speed :     {d}\n\
            position : [{:2f},{:2f}]\n\
            ------------------------------\n".format(\
                self.button1,self.direction,self.speed,\
                self.position[0],self.position[1]))


class Robot(object):
    def __init__(self):
        self.sensors = Sensors()
        self.actuators = Actuators()
        self.infos = Infos()

    def set_speeds(self, speedl,speedr):
        self.actuators.motors.set_speeds(speedl,speedr);

    def print_infos(self):
        self.infos.print_infos()