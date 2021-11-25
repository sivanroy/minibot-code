import RPi.GPIO as GPIO

class Robot(object):
    def __init__(self):
        self.motors = Motors()
        self.buttons = Buttons()
        self.infos = Infos()

    def rien(self, speed):
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