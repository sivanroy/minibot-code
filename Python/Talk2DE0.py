#Talk2DE0.py
#import thread
import math
from controller import *
import threading 
import spidev
import time

D_odo = 0.045           #odometer_diameter
D_wheel = 0.06          #wheels diameter 


""" 
Function running in paralel into a thread to actualize PID 
Controller with his new values

Take Mycontroller in arg;
run untill thread_exit =1
"""
def talk(MyController):
     #debug parameters
    verbose = 0
    Encoder = 1

    DE02RPI = MyController.DE02RPI 
    PID_obj = MyController.PID_obj
    while(MyController.thread_exit == 0):
        #take odo values
        omega_mes_r = DE02RPI.dright(Encoder) #cst from controller.py 
        omega_mes_l = DE02RPI.dleft(Encoder)
        if (verbose):
            print("omega_mes_r = {} \n omega_mes_l = {} \n"\
                .format(omega_mes_r,omega_mes_l) )
        #PID
        PID_obj.set_mes(omega_mes_l,omega_mes_r)
        #actual position
        MyController.compute_update_pos(omega_mes_l,omega_mes_r)
        #wheels
        MyController.send_to_motors()
        #if (sleep): 
        #    time.sleep(1e-3) #half of frequence of odo ?


"""
Class for multi-thread
Highly inspired of "#https://www.tutorialspoint.com/
        python/python_multithreading.htm"
"""
class myThread (threading.Thread):
   def __init__(self, func, arg1, threadID):
      threading.Thread.__init__(self)
      self.threadID = threadID
      self.status = 0
      self.arg = arg1 #if need more than 1 arg, make a list
      self.func = func

   def run(self):
      print("Starting : threadID = {}\n".format(self.threadID))
      self.func(self.arg)
      print("Exiting  : threadID = {}\n".format(self.threadID))


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
        self.thread = myThread(talk,self.MyController,0) 
        
    def start_thread(self):
        self.thread.start()

    def count(self,spi):
        treshold = 4192
        return spi[4] + (spi[3] << 8) + (spi[2] << 16) + (spi[1] << 24) - treshold

    def dright(self,Encoder = 0):
        verbose = 0
        Adr = 0x03
        D = D_odo
        if (Encoder == 1):
            Adr = 0x01
            D = D_wheel
        ToSPI_right = [Adr, 0x00, 0x00, 0x00, 0x00]
        countRight = self.count(self.MySPI_FPGA.xfer2(ToSPI_right))
        if (verbose): print(countRight)
        return countRight / (2048*4) * 2 * math.pi * D/2

    def dleft(self, Encoder = 0):
        verbose = 0
        Adr = 0x02
        D = D_odo 
        if (Encoder == 1):
            Adr = 0x00
            D = D_wheel
        ToSPI_left = [Adr, 0x00, 0x00, 0x00, 0x00]
        countLeft = -1 * self.count(self.MySPI_FPGA.xfer2(ToSPI_left))
        if (verbose) : print(countLeft)
        return countLeft  / (2048*4) * 2 * math.pi * D/2

