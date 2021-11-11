from rplidar import RPLidar
import RPi.GPIO as GPIO
import time

ENA1, ENA2, DIR1, DIR2 = 12, 13, 5, 6
GPIO.setmode(GPIO.BCM)
GPIO.setup(ENA1,GPIO.OUT)  #PWM0 - M1PWM (https://www.etechnophiles.com/wp-content/uploads/2021/01/R-Pi-4-GPIO-Pinout.jpg)
GPIO.setup(ENA2,GPIO.OUT)  #PWM1 - M2PWM
GPIO.setup(DIR1,GPIO.OUT)   #M1DIR
GPIO.setup(DIR2,GPIO.OUT)   #M2DIR

PWM1 = GPIO.PWM(ENA1,20000)# left wheel                 #(pin, pwm freq)
PWM2 = GPIO.PWM(ENA2,20000)# right wheel                 #max freq = 20k (https://www.pololu.com/product/1213)
PWM1.start(0)
PWM2.start(0)

lidar = RPLidar('/dev/ttyUSB0')
info = lidar.get_info()
print(info)


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
            if (dist_max<200): #30cm
                PWM1.ChangeDutyCycle(0)
                PWM2.ChangeDutyCycle(0)
                print('stop')
            elif(dist_max > 700):
                PWM1.ChangeDutyCycle(0)
                PWM2.ChangeDutyCycle(0)
                print('noting')      
                
            else:
                print(theta_max)
                if (theta_max > 180+error) :
                    GPIO.output(DIR1, GPIO.LOW)
                    PWM1.ChangeDutyCycle(2)
                    GPIO.output(DIR2, GPIO.HIGH)
                    PWM2.ChangeDutyCycle(2)
                    print('+')
                elif (theta_max < 180-error):
                    GPIO.output(DIR1, GPIO.HIGH)
                    PWM1.ChangeDutyCycle(2)
                    GPIO.output(DIR2, GPIO.LOW)
                    PWM2.ChangeDutyCycle(2)
                    print('-')
                else :
                    GPIO.output(DIR1, GPIO.HIGH)
                    PWM1.ChangeDutyCycle(2)
                    GPIO.output(DIR2, GPIO.HIGH)
                    PWM2.ChangeDutyCycle(2)
                    print('equ')
            dist_max = 0
            theta_max = 0
            
        else: #not a tour done
            if (dist_max < dist):
                dist_max = dist
                theta_max = angle
                
        prev_angle = angle

"""
lidar.stop()
lidar.stop_motor()
lidar.disconnect()
"""