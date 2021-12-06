import RPi.GPIO as GPIO
from time import sleep
import spidev


MySPI_FPGA = spidev.SpiDev()
MySPI_FPGA.open(0,1)
MySPI_FPGA.max_speed_hz = 500000

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)


def count(spi):
    treshold = 4192

    return spi[4] + (spi[3] << 8) + (spi[2] << 16) + (spi[1] << 24) - treshold


while 1:
    ToSPI_leftEnc = [0x00, 0x00, 0x00, 0x00, 0x00]
    countLeftEnc = -1 * count(MySPI_FPGA.xfer2(ToSPI_leftEnc))
    # -1 car à gauche on est en inversé

    ToSPI_rightEnc = [0x01, 0x00, 0x00, 0x00, 0x00]
    countRightEnc = count(MySPI_FPGA.xfer2(ToSPI_rightEnc))

    ToSPI_leftOdo = [0x02, 0x00, 0x00, 0x00, 0x00]
    countLeftOdo = -1 * count(MySPI_FPGA.xfer2(ToSPI_leftOdo))

    ToSPI_rightOdo = [0x03, 0x00, 0x00, 0x00, 0x00]
    countRightOdo = count(MySPI_FPGA.xfer2(ToSPI_rightOdo))
    
    if (countLeftOdo != 0):
        print("countLeftOdo =", countLeftOdo)
    if (countRightOdo != 0):
        print("countRightOdo =", countRightOdo)
    if (countLeftEnc != 0):
        print("countLeftEnc =", countLeftEnc)
    if (countRightEnc != 0):
        print("countRightEnc =", countRightEnc)




