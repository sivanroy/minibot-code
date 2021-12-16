from minibot import *
from rplidar import RPLidar
from time import sleep


#begin lidars
lidar = RPLidar('/dev/ttyUSB0')
lidar.start_motor()
info = lidar.get_info()
print(info)

minibot = Minibot()


def headquarters(scan_data):
    WhatToDo = buttonON(minibot)
    if WhatToDo == -1:
        minibot.shutdown()
        return -1
    elif WhatToDo == 1:
        minibot.follow(scan_data)
    return 1

#begin scan
for i, lidar_scan in enumerate(lidar.iter_scans()):
    #set scan data
    theta = []; dist = []
    for scan in (lidar_scan):  # each scan = [quality,angle,dist]
        theta.append(scan[1])
        dist.append(scan[2])
    WhatToDo = headquarters([dist, theta])
    sleep(0.0005)
    if WhatToDo == -1:
        break

lidar.stop()
lidar.stop_motor()
lidar.disconnect()

#minibot.plot_w()