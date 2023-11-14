import RPi.GPIO as GPIO
import time

pin_numbers = [38, 37, 36, 35, 33, 31]

GPIO.setwarnings(False) # Ignore warning for now
GPIO.setmode(GPIO.BOARD) # Use physics pin numbering
for pin_number in pin_numbers:
	GPIO.setup(pin_number, GPIO.IN, pull_up_down=GPIO.PUD_DOWN) # Input; pulled low (off)

while True:
	for pin_number in pin_numbers:
		if GPIO.input(pin_number) == GPIO.HIGH:
			print("Button %i is pushed!"%pin_number)
		else:
			print("Button %i is off."%pin_number)

	time.sleep(0.1)
