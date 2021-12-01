import RPi.GPIO as GPIO
from displacement import motors, MAX_SPEED

class Sensors(object):
    def __init__(self):
        self.buttons = Buttons();
        self.lidar = None

class Button(object):
    def __init__(self,pin):
        self.ON = 0
        self.pin = pin
        self.countPush = 0
        GPIO.setwarnings(False) # Ignore warning for now
        GPIO.setup(pin, GPIO.IN, pull_up_down=GPIO.PUD_DOWN) # Set pin 10 to be an input pin and set initial value to be pulled low (off)
        GPIO.add_event_detect(self.pin,GPIO.RISING,callback=button_callback) 

    def button_callback(self,channel):
        print("Button was pushed!")
        self.ON = 1
        self.countPush+=1

    def isPushed(self):
        return self.ON

    def count(self):
        return self.countPush

    def clear(self):
        self.ON = 0

    def print_infos(self):
        print("------------------------------\n\
            pin = {}\n\
            ON =  {}\n".format(self.pin,self.ON))


class Buttons(object):
    PIN = 26 #BCM
    def __init__(self):
        self.button1 = Button(PIN)

    def print_infos(self):
        print("Button #1 :\n")
        self.button1.print_infos()



class Actuators(object):
    def __init__(self):
        self.motors = Motors()

    def set_speeds(self,speedl,speedr):
        self.motors.set_speeds(speedl,speedr)


class Infos(object):
    def __init__(self):
        self.direction = 0
        self.speed = 0
        self.position = [0,0]    
    def print_infos(self):
        print("------------------------------\n\
            button1 :   {d}\n\
            direction : {d}\n\
            speed :     {d}\n\
            position : [{:2f},{:2f}]\n\
            ------------------------------\n".format(\
                self.button1,self.direction,self.speed,\
                self.position[0],self.position[1]))


class Robot(object):
    def __init__(self):
        self.ON = 0
        self.sensors = Sensors()
        self.actuators = Actuators()
        self.infos = Infos()

    def set_speeds(self, speedl,speedr):
        self.actuators.motors.set_speeds(speedl,speedr);

    def print_infos(self):
        self.infos.print_infos()

    def activate(self):
        self.ON = 1

    def shutdown(self):
        self.ON = 0