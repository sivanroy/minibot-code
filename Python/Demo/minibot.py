import RPi.GPIO as GPIO
from displacement import *


class minibot(object):
    def __init__(self):
        self.motors = Motors()
        self.motors.start_all()
