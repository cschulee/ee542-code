#! user/bin/python
#  Raspberry Pi Coordinated Load Carriage Swarm

# Import required modules
from hmc5883l import hmc5883l
from nrf24pihub import NRF24
import numpy as np
import RPi.GPIO as GPIO
import picamera, cv2, diff_drv, threading, time

# Global data
global heading
global heading_rate
global heading_tgt
global speed
global speed_tgt
global compass
global compass_enable
global mode

# Initialize peripherals
compass = hmc5883l()
camera  = picamera.PiCamera()
drive   = diff_drv.diff_drv(7,11,13,15,16,18,160)

radio   = NRF24()
pipes = [[0xf0, 0xf0, 0xf0, 0xf0, 0xe1], [0xf0, 0xf0, 0xf0, 0xf0, 0xd2]]
radio.begin(0, 0,25,18) #set CE as 25, IRQ as 18
radio.setRetries(15,15)
radio.setPayloadSize(32)
radio.setChannel(0x4c)
radio.setDataRate(NRF24.BR_250KBPS)
radio.setPALevel(NRF24.PA_MAX)
radio.setAutoAck(1)
radio.openWritingPipe(pipes[0])
radio.openReadingPipe(1, pipes[1])
radio.startListening()

# Initialize interrupts
GPIO.add_event_detect(15, GPIO.FALLING, callback = mot_cb)
GPIO.add_event_detect(18, GPIO.FALLING, callback = msg_cb)

# Define interrupt callbacks and compass thread
def mot_cb():
    #TODO what should be done when motion is detected

def msg_cb():
    #TODO expand what should be done when message is received
    recv_buffer = []
    radio.read(recv_buffer)
    out = ''.join(chr(i) for i in recv_buffer)
    print out
    #Clear IRQ
    radio.write_register(radio.STATUS,
                         radio.read_register(radio.STATUS)| 0b01000000)

def get_heading():
    global compass
    global heading
    global heading_rate
    heading_pv = 0    # First time through
    DELTAT     = 0.04 # 25 HZ
    while compass_enable == True:
        heading = compass.heading()
        heading_rate = (heading - heading_pv)/DELTAT
        heading_pv = heading
        time.sleep(DELTAT)

# Start compass thread
compass_enable = True
compassThread = threading.Thread(target=get_heading,name='compassThread')
compassThread.start

# Define list of modes
modes = {0 : wait,
         1 : align,
         2 : assemble,
         3 : track,
         4 : master}

def wait():
    global mode
    drive.coast()
    while mode == 0:
        time.sleep(1)

def align():
    global mode
    while mode == 1:
        #TODO align task

def assemble(pos):
    global mode
    while mode == 2:
        # TODO assemble task

def track():
    global mode
    while mode == 3:
        # TODO track task

def master():
    global mode
    while mode == 4:
        # TODO master task


# Perform main program
# Enter WAIT mode
mode = 0
modes[mode]()










