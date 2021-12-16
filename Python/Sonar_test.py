import spidev
import time
import RPi.GPIO as GPIO

MySPI_FPGA = spidev.SpiDev()
MySPI_FPGA.open(0, 1)
MySPI_FPGA.max_speed_hz = 500000
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)


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


Sonar_test()

