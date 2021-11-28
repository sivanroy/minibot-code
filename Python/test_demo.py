from rplidar import RPLidar
import RPi.GPIO as GPIO
import time
import numpy as np
#Used
from displacement import motors, MAX_SPEED
import controller as c
import button
from button import BT

#begin motors
motors.start_all()

#begin lidars
lidar = RPLidar('/dev/ttyUSB0')
lidar.start_motor()
info = lidar.get_info()
print(info)

#begin scan
for i, lidar_scan in enumerate(lidar.iter_scans()):
    #set scan data
    theta = []; dist = []
    for scan in (lidar_scan):  # each scan = [quality,angle,dist]
        theta.append(scan[1])
        dist.append(scan[2])
    info_c = c.controller([dist,theta],BT)
    if (info_c == -1):
        break


lidar.stop()
lidar.stop_motor()
lidar.disconnect()







