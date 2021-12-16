import spidev
from minibot import *
import time
import RPi.GPIO as GPIO

minibot = Minibot()

MySPI_FPGA = spidev.SpiDev()
MySPI_FPGA.open(0, 1)
MySPI_FPGA.max_speed_hz = 500000
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

def headquarters():
    WhatToDo = buttonON(minibot)
    if WhatToDo == -1:
        minibot.shutdown()
        return -1
    elif WhatToDo == 1:
        minibot.calib(MySPI_FPGA)
    return 1

def count(spi, tr):
    if tr:
        treshold = 4192
    else:
        treshold = 0
    value = spi[4] + (spi[3] << 8) + (spi[2] << 16) + (spi[1] << 24) - treshold
    return value


def DataFromDE0():
    ToSPI_Sonar = [0x04, 0x00, 0x00, 0x00, 0x00]
    countSonar = count(MySPI_FPGA.xfer2(ToSPI_Sonar), 0)
    return countSonar

def DataEnc():
    ToSPI_rightEnc = [0x01, 0x00, 0x00, 0x00, 0x00]
    countRightEnc = count(MySPI_FPGA.xfer2(ToSPI_rightEnc), 1)

    ToSPI_leftEnc = [0x00, 0x00, 0x00, 0x00, 0x00]
    countLeftEnc = count(MySPI_FPGA.xfer2(ToSPI_leftEnc), 1)

    return countRightEnc, countLeftEnc


def Sonar_test():

    while 1:
        cnt = DataFromDE0()
        #if cnt != 0:
        print(cnt)


def Calib_enc():
    while 1:
        WhatToDo = headquarters()
        if WhatToDo == -1:
            print(minibot.return_count())
            break



#Sonar_test()
Calib_enc()

"""
30 cm
2pir = 18.85cm
30/18.85 = 1.59

1024*4 = 4096
4096*1.59 = 6512

dt = 1ms
6914,7206
6000,6111
5421, 5472
6929,7013
6197,6231
6400,6544
6062, 6129
6346, 6248

dt = 2ms
8192*1.59 = 13025

12717, 12603
13417,13226
13008
"""