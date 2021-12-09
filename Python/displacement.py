import RPi.GPIO as GPIO

MAX_SPEED = 100
FREQ = 1e3

def io_init():
    PWM1, PWM2, DIR1, DIR2 = 12, 13, 5, 6
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(PWM1, GPIO.OUT)
    GPIO.setup(PWM2, GPIO.OUT)
    GPIO.setup(DIR1, GPIO.OUT)
    GPIO.setup(DIR2, GPIO.OUT)

class Motor(object):
    MAX_SPEED = 100
    FREQ = 1e3

    def __init__(self, PWM_PIN, DIR_PIN):
        io_init()
        self.PWM = GPIO.PWM(PWM_PIN, FREQ)
        self.PWM_PIN = PWM_PIN
        self.DIR_PIN = DIR_PIN

    def start(self):
        self.PWM.start(0)

    def set_speed(self, speed):
        if speed < 0:
            speed = -speed
            dir_value = 0
        else:
            dir_value = 1

        if speed > MAX_SPEED:
            speed = MAX_SPEED

        GPIO.output(self.DIR_PIN, dir_value)
        self.PWM.ChangeDutyCycle(speed)

    def stop(self):
        self.PWM.ChangeDutyCycle(0)


class Motors(object):
    MAX_SPEED = 100

    def __init__(self):
        self.motor_l = Motor(12, 5)  # left  wheel
        self.motor_r = Motor(13, 6)  # right wheel

    def start_all(self):
        self.motor_l.start()
        self.motor_r.start()

    def stop_all(self):
        self.motor_l.stop()
        self.motor_r.stop()

    def set_speeds(self, speed_l, speed_r):
        self.motor_l.set_speed(speed_l)
        self.motor_r.set_speed(speed_r)
