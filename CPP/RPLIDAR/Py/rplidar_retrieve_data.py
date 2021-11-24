from rplidar import RPLidar
import RPi.GPIO as GPIO
import time

lidar = RPLidar('/dev/ttyUSB0')
info = lidar.get_info()
print(info)


def scan_lidar():
    for i, scan in enumerate(lidar.iter_scans()): #each scan = [quality,angle,dist]
        prev_angle = 0
        result = [[],[]]  #[[dist][angle]]
        (quality,angle,dist) = scan[i]
        if(prev_angle-angle > 100 ): # after one tour, do smthg
            return result
        else: #not a tour done
            result[0].append(dist)
            result[1].append(angle)
        prev_angle = angle