import RPi.GPIO as GPIO

PWM11, PWM2, DIR1, DIR2 = 12, 13, 5, 6
GPIO.setmode(GPIO.BCM)
GPIO.setup(ENA1,GPIO.OUT)
GPIO.setup(ENA2,GPIO.OUT)
GPIO.setup(DIR1,GPIO.OUT)
GPIO.setup(DIR2,GPIO.OUT)

PWM1 = GPIO.PWM(ENA1,20000)# left wheel                 #(pin, pwm freq)
PWM2 = GPIO.PWM(ENA2,20000)# right wheel                 #max freq = 20k (https://www.pololu.com/product/1213)
PWM1.start(0)
PWM2.start(0)

print("a")