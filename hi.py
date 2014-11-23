import RPi.GPIO as GPIO
import time
GPIO.setwarnings(False)
GPIO.setmode (GPIO.BOARD)

GPIO.setup(22, GPIO.OUT)
GPIO.setup(24, GPIO.OUT)
GPIO.setup(26, GPIO.OUT)
GPIO.setup(19, GPIO.OUT)
GPIO.setup(21, GPIO.OUT)
GPIO.setup(23, GPIO.OUT)
p = GPIO.PWM(22,185)
c = GPIO.PWM(19,150)
p.start(0)
c.start(0)

try:
	while True:
	   #for i in range(100):
		
		#time.sleep(0.2)
		c.ChangeDutyCycle(40)
		GPIO.output(21,1)
		GPIO.output(23,1)
                p.ChangeDutyCycle(60)
		GPIO.output(24,0)
		GPIO.output(26,1)
		#time.sleep(0.2)
	   #for i in range(100):
		 #p.ChangeDutyCycle(100-i)
		 #time.sleep(0.2)
		 #c.ChangeDutyCycle(i)
	       	 #time.sleep(0.2)
		
except KeyboardInterrupt:
	pass

p.stop()
#c.stop()

GPIO.cleanup()
