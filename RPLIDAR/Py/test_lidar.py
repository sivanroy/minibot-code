from rplidar import RPLidar
import RPi.GPIO as GPIO
import time
import numpy as np


MAXDIST = 2000
MINDIST = 200
ERROR = 30
DC = 0.5

ENA1, ENA2, DIR1, DIR2 = 12, 13, 5, 6
GPIO.setmode(GPIO.BCM)
GPIO.setup(ENA1,GPIO.OUT)  #PWM0 - M1PWM (https://www.etechnophiles.com/wp-content/uploads/2021/01/R-Pi-4-GPIO-Pinout.jpg)
GPIO.setup(ENA2,GPIO.OUT)  #PWM1 - M2PWM
GPIO.setup(DIR1,GPIO.OUT)   #M1DIR
GPIO.setup(DIR2,GPIO.OUT)   #M2DIR

PWM1 = GPIO.PWM(ENA1,20000)# left wheel                 #(pin, pwm freq)
PWM2 = GPIO.PWM(ENA2,20000)# right wheel                 #max freq = 20k (https://www.pololu.com/product/1213)
PWM1.start(0)
PWM2.start(0)

lidar = RPLidar('/dev/ttyUSB0')
info = lidar.get_info()
print(info)

def left_wheel(on,high=1):
    if (high):
        GPIO.output(DIR2, GPIO.HIGH)
    else:
        GPIO.output(DIR2, GPIO.LOW)
    if (on):
        PWM1.ChangeDutyCycle(DC)
    else:
        PWM1.ChangeDutyCycle(0)
        #GPIO.output(DIR2, GPIO.LOW)


def right_wheel(on,high=1):
    if (high):
        GPIO.output(DIR1, GPIO.HIGH)
    else:
        GPIO.output(DIR1, GPIO.LOW)
    if (on):
        PWM2.ChangeDutyCycle(DC)
        GPIO.output(DIR1, GPIO.HIGH)
    else:
        PWM2.ChangeDutyCycle(0)
        #GPIO.output(DIR2, GPIO.LOW)

for i,lidar_scan in enumerate(lidar.iter_scans()):
    theta = []
    dist  = []
    #print('%d: Got %d measurments' % (i, len(lidar_scan)))
    for scan in (lidar_scan): #each scan = [quality,angle,dist]
        theta.append(scan[1])
        dist.append(scan[2])
    max_index= np.where(dist == np.amin(np.array(dist)))[0][0]
    dist_p = dist[max_index]
    theta_p = theta[max_index]

    if (dist_p < MINDIST or dist_p > MAXDIST):
        left_wheel(0)
        right_wheel(0)
        #print('stop')
    elif (theta_p < 180-ERROR):
        left_wheel(0)
        right_wheel(1)
        #print('+')
    elif (theta_p > 180+ERROR):
        left_wheel(1)
        right_wheel(0)
        #print('-')
    else:
        left_wheel(1)
        right_wheel(1) 
        #print('straigth')


"""
lidar.stop()
lidar.stop_motor()
lidar.disconnect()
"""





