import time
import RPi.GPIO as GPIO


GPIO.setmode(GPIO.BOARD)
GPIO.setup(12, GPIO.OUT)

pwm = GPIO.PWM(12, 1000)
dc=0				#duty cycle
pwm.start(dc)

try:
	while True:
		for dc in range(0, 100, 1):
			pwm.ChangeDutyCycle(dc)
			time.sleep(0.05) #wait for .05 sec
			print(dc)
		for dc in range(100, -1, -1):
			pwm.ChangeDutyCycle(dc)
			time.sleep(0.05)
			print(dc)
		time.sleep(5)
except:
	print('ctrl-c pressed')

pwm.stop()		#stop PWM
GPIO.cleanup()
