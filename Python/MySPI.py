import RPi.GPIO as GPIO
from time import sleep
import spidev

MyARM_ResetPin = 19 # Pin 4 of connector = BCM19 = GPIO[1]

MySPI_FPGA = spidev.SpiDev()
MySPI_FPGA.open(0,0)
MySPI_FPGA.max_speed_hz = 50000

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup(MyARM_ResetPin, GPIO.OUT)

GPIO.output(MyARM_ResetPin, GPIO.HIGH)
sleep(0.1)
GPIO.output(MyARM_ResetPin, GPIO.LOW)
sleep(0.1)

i = 0
stop = 4000
while i < stop:
    ToSPI_leftEnc = [0x00, 0x00, 0x00, 0x00, 0x00]
    countLeftEnc = MySPI_FPGA.xfer2(ToSPI_leftEnc)

    ToSPI_rightEnc = [0x01, 0x00, 0x00, 0x00, 0x00]
    countRightEnc = MySPI_FPGA.xfer2(ToSPI_rightEnc)

    ToSPI_leftOdo = [0x02, 0x00, 0x00, 0x00, 0x00]
    countLeftOdo = MySPI_FPGA.xfer2(ToSPI_leftOdo)

    ToSPI_rightOdo = [0x03, 0x00, 0x00, 0x00, 0x00]
    countRightOdo = MySPI_FPGA.xfer2(ToSPI_rightOdo)

    print("countLeftOdo =", countLeftOdo)

    i += 1


