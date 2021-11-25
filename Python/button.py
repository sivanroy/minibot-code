#button.py
#https://raspberrypihq.com/use-a-push-button-with-raspberry-pi-gpio/
import RPi.GPIO as GPIO # Import Raspberry Pi GPIO library

BT = 0


def button_callback(channel):
    print("Button was pushed!")
    BT = 1

PIN = 37
GPIO.setwarnings(False) # Ignore warning for now
GPIO.setmode(GPIO.BOARD) # Use physical pin numbering
GPIO.setup(PIN, GPIO.IN, pull_up_down=GPIO.PUD_DOWN) # Set pin 10 to be an input pin and set initial value to be pulled low (off)
GPIO.add_event_detect(PIN,GPIO.RISING,callback=button_callback) # Setup event on pin 10 rising edge

if (BT == 1) :
	print("bt settled to 0")
	BT = 0
	
message = input("Press enter to quit\n\n") # Run until someone presses enter


GPIO.cleanup() # Clean up