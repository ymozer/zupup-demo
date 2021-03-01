import RPi.GPIO as GPIO


GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup(17, GPIO.IN)
GPIO.setup(18, GPIO.OUT)
pwm = GPIO.PWM(18, 10)
pwm.stop()

GPIO.cleanup()
