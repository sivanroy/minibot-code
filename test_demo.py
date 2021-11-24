from rplidar import RPLidar
import RPi.GPIO as GPIO
import time
import numpy as np
from Displacement import motors, MAX_SPEED

MAXDIST = 600
MINDIST = 300
ERROR = 20
speed = 0.5

motors.start_all()

lidar = RPLidar('/dev/ttyUSB0')
info = lidar.get_info()
print(info)

for i, lidar_scan in enumerate(lidar.iter_scans()):
    theta = []
    dist = []
    # print('%d: Got %d measurments' % (i, len(lidar_scan)))
    for scan in (lidar_scan):  # each scan = [quality,angle,dist]
        theta.append(scan[1])
        dist.append(scan[2])
    max_index = np.where(dist == np.amin(np.array(dist)))[0][0]
    dist_p = dist[max_index]
    theta_p = theta[max_index]

    # Balise trop proche -> stop
    if (dist_p < MINDIST or dist_p > MAXDIST):
        motors.stop_all()
        # print('stop')

    # Balise à gauche
    elif (theta_p < 180 - ERROR):

        # Balise arriere gauche
        if (theta_p < 90 - ERROR):
            motors.set_speeds(-speed, speed)
            # print('+ arr')

        # Balise avant gauche
        else:
            motors.set_speeds(0, speed)
            # print('+ avt')

    # Balise à droite
    elif (theta_p > 180 + ERROR):

        # Balise arriere droite
        if (theta_p > 270 - ERROR):
            motors.set_speeds(speed, -speed)
            # print('- arr')

        # Balise avant gauche
        else:
            motors.set_speeds(speed, 0)
            # print('- avt')

    # Balise en face
    else:
        motors.set_speeds(speed, speed)
        # print('straigth')

"""
lidar.stop()
lidar.stop_motor()
lidar.disconnect()
"""






