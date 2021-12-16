#Talk2DE0.py
#import thread
import math
from controller import *
import threading 
import spidev
import time

D_odo = 0.045           #odometer_diameter
D_wheel = 0.06          #wheels diameter 
deltat = 2e-3

"""
Use to talk btwn DE0 and RPI through SPI communication

use dright to have mesured distance/Deltat of right odo
use dleft  to have mesured distance/Deltat of left  odo
"""
class DE02Rpi(object):
    def __init__(self,MyController):
        self.MyController = MyController
        self.MySPI_FPGA = spidev.SpiDev()
        self.MySPI_FPGA.open(0,1)
        self.MySPI_FPGA.max_speed_hz = 500000
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)        

    def count(self,spi,verbose=0):
        treshold = 4192
        value = spi[4] + (spi[3] << 8) + (spi[2] << 16) + (spi[1] << 24) - treshold
        if (value == - treshold):
            if (verbose) :
                print("Maybe is there an error (: Did you Program the DE0?")
        return value

    """
    Give the mesure of angular speed of the odo/encoder

    GIVE THE DELTATHETA !!! ???
    """
    def mes_right(self,Encoder = 0,verbose = 0):
        Adr = 0x03
        D = D_odo
        if (Encoder == 1):
            Adr = 0x01
            D = D_wheel
        ToSPI_right = [Adr, 0x00, 0x00, 0x00, 0x00]
        countRight = self.count(self.MySPI_FPGA.xfer2(ToSPI_right),verbose=verbose)
        if (verbose): print(countRight)
        return countRight * 1/deltat / 1920 * 2 * math.pi # * Deltat

    def mes_left(self, Encoder = 0,verbose = 0):
        Adr = 0x02
        D = D_odo 
        if (Encoder == 1):
            Adr = 0x00
            D = D_wheel
        ToSPI_left = [Adr, 0x00, 0x00, 0x00, 0x00]
        countLeft = -1 * self.count(self.MySPI_FPGA.xfer2(ToSPI_left),verbose=verbose)
        if (verbose) : print(countLeft)
        return countLeft /deltat / 1920 * 2 * math.pi# * Deltat #????

