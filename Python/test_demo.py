from rplidar import RPLidar
import RPi.GPIO as GPIO
import time
import numpy as np
from displacement import motors, MAX_SPEED
import controller as c

speed = 0.5

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
    c.controller([dist,theta])


lidar.stop()
lidar.stop_motor()
lidar.disconnect()







