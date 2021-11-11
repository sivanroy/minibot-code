import RPi.GPIO as GPIO

class Motor(object):

    def __init__(self, PWM_PIN, DIR_PIN):
        self.PWM = PWM_PIN
        self.DIR = DIR_PIN

    def init(self):
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.PWM, GPIO.OUT)
        GPIO.setup(self.DIR, GPIO.OUT)

        PWM1 = GPIO.PWM(PWM1, 20000)  # left wheel
        PWM2 = GPIO.PWM(PWM2, 20000)  # right wheel

def motor_init():
    PWM1, PWM2, DIR1, DIR2 = 12, 13, 5, 6
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(PWM1,GPIO.OUT)
    GPIO.setup(PWM2,GPIO.OUT)
    GPIO.setup(DIR1,GPIO.OUT)
    GPIO.setup(DIR2,GPIO.OUT)

    PWM1 = GPIO.PWM(PWM1,20000)# left wheel
    PWM2 = GPIO.PWM(PWM2,20000)# right wheel
    PWM1.start(0)
    PWM2.start(0)

print("a")