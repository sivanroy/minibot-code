from rplidar import RPLidar
import RPi.GPIO as GPIO
import time

lidar = RPLidar('/dev/ttyUSB0')
info = lidar.get_info()
print(info)

def scan_lidar():
    for i, scan in enumerate(lidar.iter_scans()): #each scan = [quality,angle,dist]
        theta_max = 0
        dist_max = 0
        i = 0
        prev_angle = 0
        
        
        while (i < len(scan)):
            (quality,angle,dist) = scan[i]
            i+=1
            error = 20
            if(prev_angle-angle > 1): # after one tour, do smthg
                dist_max = 0
                theta_max = 0  
            else: #not a tour done
                if (dist_max < dist):
                    dist_max = dist
                    theta_max = angle 
            prev_angle = angle