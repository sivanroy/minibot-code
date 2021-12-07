#Talk2DE0.py
#import thread
from controller import *
import threading 
#https://www.tutorialspoint.com/python/python_multithreading.htm
import time


def talk(MyController):
	DE02RPI = MyController.DE02RPI 
	PID_obj = MyController.PID_obj
	while(self.thread_exit == 0):
		#take odo values
		omega_mes_r = DE02RPI.dright() #cst from controller.py 
		omega_mes_l = DE02RPI.dleft()
		#PID
		set_mes(omega_mes_l,omega_mes_r)
		#actual position
		set_pos(omega_mes_l,omega_mes_r)
		#wheels
		MyController.send_to_motors()
		time.sleep(0.5e-3) #half of frequence of odo ?


class myThread (threading.Thread):
   def __init__(self, func, arg1, threadID):
      threading.Thread.__init__(self)
      self.threadID = threadID
      self.status = 0
      self.arg = arg1 #if need more than 1 arg, make a list

   def run(self):
      print "Starting " + self.threadID
      func(self.arg)
      print "Exiting " + self.threadID


class DEO2Rpi(object):
	def __init__(self,MyController):
		self.MyController = MyController
        MySPI_FPGA = spidev.SpiDev()
        MySPI_FPGA.open(0,1)
        MySPI_FPGA.max_speed_hz = 500000
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
		self.thread = myThread(1) 
		self.thread.start(talk)

    def count(spi):
        treshold = 4192
        return spi[4] + (spi[3] << 8) + (spi[2] << 16) + (spi[1] << 24) - treshold

	def dright(self):
        ToSPI_rightOdo = [0x03, 0x00, 0x00, 0x00, 0x00]
        countRightOdo = count(MySPI_FPGA.xfer2(ToSPI_rightOdo))
        return countRightOdo

	def dleft(self):
		ToSPI_leftOdo = [0x02, 0x00, 0x00, 0x00, 0x00]
        countLeftOdo = -1 * count(MySPI_FPGA.xfer2(ToSPI_leftOdo))
        return countLeftOdo
