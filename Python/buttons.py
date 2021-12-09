import RPi.GPIO as GPIO
from robot import *
class Button(object):
    def __init__(self,pin):
        self.ON = 0
        self.pushed = 0 #has been pushed
        self.pin = pin
        self.countPush = 0
        GPIO.setwarnings(False) # Ignore warning for now
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(pin, GPIO.IN, pull_up_down=GPIO.PUD_DOWN) # Set pin 10 to be an input pin and set initial value to be pulled low (off)
        GPIO.add_event_detect(self.pin,GPIO.FALLING,callback=self.button_callback,bouncetime=200)#bouncetime in ms 

    def button_callback(self,channel):
        print("Button was pushed!\n")
        self.ON = 1
        self.countPush+=1
        print("countPush = {}\n".format(self.countPush))

    def wasPushed(self): #has been pushed
        return (self.ON)

    def count(self):
        return self.countPush

    def clear(self):
        self.ON = 0

    def print_infos(self):
        print("------------------------------\n\
            pin = {}\n\
            ON =  {}\n\
            ------------------------------\n".format(self.pin,self.ON))


class Buttons(object):
    PIN = 26 #BCM
    def __init__(self):
        self.button1 = Button(26)

    def print_infos(self):
        print("Button #1 :\n")
        self.button1.print_infos()

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