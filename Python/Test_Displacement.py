from displacement import *
import time

def test():

    motors = Motors()

    speed = 1
    motors.start_all()

    motors.set_speeds(speed, speed)

    time.sleep(2)

    motors.stop_all()

    time.sleep(1)

    motors.set_speeds(-speed, -speed)

    time.sleep(2)

    motors.set_speeds(speed, -speed)

    time.sleep(2)

    motors.stop_all()


test()
