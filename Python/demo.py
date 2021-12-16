from rplidar import RPLidar
import RPi.GPIO as GPIO
import time
import numpy as np
import headquarters as h
from robot import *

#begin motors
#motors.start_all()

#begin lidars
lidar = RPLidar('/dev/ttyUSB0')
lidar.start_motor()
info = lidar.get_info()
print(info)

MyRobot = Robot()

#begin scan
for i, lidar_scan in enumerate(lidar.iter_scans()):
    #set scan data
    theta = []; dist = []
    for scan in (lidar_scan):  # each scan = [quality,angle,dist]
        theta.append(scan[1])
        dist.append(scan[2])
    info_c = h.headquarters([dist,theta],MyRobot)
    if (info_c == -1):
        break

lidar.stop()
lidar.stop_motor()
lidar.disconnect()







