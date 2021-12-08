import numpy as np
import RPi.GPIO as GPIO
from robot import *
import time
from controller import *
from Talk2DE0 import *

import spidev



MyARM_ResetPin = 19 # Pin 4 of connector = BCM19 = GPIO[1]

MySPI_FPGA = spidev.SpiDev()
MySPI_FPGA.open(0,0)
MySPI_FPGA.max_speed_hz = 500000

MyRobot = Robot()

ref = np.ones(400) #1sec
ref[0:100] = ref[0:100]* 0
ref[100:200] = ref[100:200]* 5
ref[200:300] = ref[200:300]* 20
ref[300:400] = ref[300:400]* 0


def buttonOn(MyRobot):
    Mybutton1 = MyRobot.sensors.buttons.button1 #gere le button
    if (Mybutton1.wasPushed() == 0):
        if(MyRobot.isON()):
            return 1
    else:
        if(Mybutton1.count()<2):
            Mybutton1.clear()
            MyRobot.activate()
            print(Mybutton1.print_infos())
            print("Robot is ON\n")
            
        else:
            Mybutton1.clear()
            MyRobot.shutdown()
            MyRobot.actuators.motors.set_speeds(0,0)
            print("stop the motors \n")
            return -1 
    return 0

MyController = MyRobot.controller
MyDE02RPI = MyController.DE02RPI
MyPID = MyController.PID_obj

i = 0
Running = 1
while(Running):
    WhatToDo = buttonOn(MyRobot)
    if (WhatToDo == -1 or i >= 400):
        Running = 0
        MyRobot.shutdown()
        print('here')
        break
    elif (WhatToDo ==  1):
        MyPID.set_setpoint_r(ref[i])
        MyPID.set_setpoint_l(ref[i])
        if (i%100 == 0): print(ref[i])
        time.sleep(0.02)
        i+=1
    else:
        print("blocked")

    