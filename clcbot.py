#! user/bin/python
#  Raspberry Pi Coordinated Load Carriage Swarm

# Import required modules
from hmc5883l import hmc5883l
from nrf24pihub import NRF24
#from adns9800 import adns9800
import numpy as np
import RPi.GPIO as GPIO
import random as rand
import picamera, cv2, diff_drv, threading, time, string, math

# Global data
global compass
global compass_enable
global mode
global master_hdg
global master_fwd
global master_rot
global reset_watchdog
global radio_payload
global players
global form
global me

# Initialize sensors
compass = hmc5883l()
camera  = picamera.PiCamera()
drive   = diff_drv.diff_drv(4,17,27,22,23,24,185)
#mouse   = adns9800()

# Initialize radio
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

# List of possible radio commands from master
global cmds
cmds = {'wait'        : wait,
        'report_id'   : report_id,
        'add_id'      : add_id,
        'update_spot' : update_spot,
        'update_form' : update_form,
        'update_hdg'  : update_hdg,
        'align'       : align,
        'assemble'    : assemble,
        'update_fwd'  : update_fwd,
        'update_rot'  : update_rot,
        'track'       : track}

# GPIO Mode - BCM
GPIO.setmode(GPIO.BCM)

# Initialize interrupts
GPIO.add_event_detect(15, GPIO.FALLING, callback = mot_cb)
GPIO.add_event_detect(18, GPIO.FALLING, callback = msg_cb)

# Define interrupt callbacks and compass thread
def mot_cb():
    print '  MOTION CALLBACK'
    #TODO what should be done when motion is detected

def msg_cb():
    global cmds
    global radio_payload
    global reset_watchdog
    recv_buffer = []
    radio.read(recv_buffer)
    msg = null_char_strip(''.join(chr(i) for i in recv_buffer))
    print '---MESSAGE RECEIVED---  ' + msg

    if msg in cmds.keys():
        reset_watchdog = 1
        cmds[msg]()
    else:
        radio_payload = msg
    
    #Clear IRQ
    radio.write_register(radio.STATUS,
                         radio.read_register(radio.STATUS)| 0b01000000)

def null_char_strip(in_str):
    return in_str.rstrip('\00')

# Define compass callback- get heading continuously
def get_heading(DELTAT):
    global compass
    global compass_enable
    global heading
    while compass_enable == True:
        heading = compass.heading()
        time.sleep(DELTAT)

def wait():
    global mode
    drive.coast()

def report_id():
    radio.write(players[0])
    radio.write('add_id')

def add_id():
    global radio_payload
    if radio_payload not in players:
        players.append(radio_payload)

def update_spot():
    global radio_payload
    global spot
    if radio_payload.find(players[0]) == 0:
        spot = int(radio_payload[-1])

def update_form():
    global radio_paylaod
    global form
    form = radio_payload

def update_hdg():
    global radio_payload
    global master_hdg
    master_hdg = float(radio_payload)

def align():
    global heading
    global master_hdg
    print '  ALIGNING TO HEADING: ' + str(master_hdg) + ' DEGREES'
    while abs(heading - master_hdg) > 5:
        drive.drive(0,10)

def assemble(pos):
    print '  GOING TO ASSEMBLY'
    radio.write(me)


def track():
    global master_fwd
    global master_rot
    print '  TRACKING FWD: ' + str(master_fwd) + ' ROT ' + str(master_rot)
    drive.drive(master_fwd,master_rot)
    
def master():
    global mode
    global heading
    global master_hdg
    global master_fwd
    global master_rot
    global players
    global me
    global form
    
    print 'MASTER MODE'

    # While not knocked off by another master:
    master_heading = heading
    while mode == 'master':

        # Wait just a second! Passivate any active bots, establish mastership
        start = time.time()
        while time.time() - start < 1:
            radio.write('wait')

        # Identify the players
        players = [me]
        start = time.time()
        while len(players) < 4 & time.time() - start < 30:
            radio.write('report_id')
        if len(players) == 4:
            form = 'quad'
        else:
            form = 'tri'

        # Update formation geometry
        start = time.time()
        while time.time() - start < 2:
            radio.write(form)
            radio.write('update_form')

        # Perform spot assignments
        start = time.time()
        while time.time() - start < 2:
            for x in range(1,len(players)):
                radio.write(players[x] + str(x))
                radio.write('update_spot')

        # 2 min align to current heading
        start = time.time()
        while time.time() - start < 2*60:
            radio.write(master_hdg)
            radio.write('update_hdg')
            radio.write('align')
            align()
            
        # Assemble into formation
        num_players = len(players)
        players = [me]
        start = time.time()
        start = time.localtime().tm_min
        while len(players) < num_players | time.time() - start < 2*60:
            radio.write('assemble')

        # Track forward 10 sec
        master_fwd = 10
        master_rot = 0        
        start = time.time()
        while time.time() - start < 10:
            radio.write(master_fwd)
            radio.write('update_fwd')
            radio.write(master_rot)
            radio.write('update_rot')
            radio.write('track')

        # Track backward 10 sec
        master_fwd = -10
        master_rot = 0        
        start = time.time()
        while time.time() - start < 10:
            radio.write(master_fwd)
            radio.write('update_fwd')
            radio.write(master_rot)
            radio.write('update_rot')
            radio.write('track')
            track()

        # Track forward right 5 sec
        master_fwd = 10
        master_rot = 5
        start = time.time()
        while time.time() - start < 5:
            radio.write(master_fwd)
            radio.write('update_fwd')
            radio.write(master_rot)
            radio.write('update_rot')
            radio.write('track')
            track()

        # Turn 180 deg and start over
        master_hdg = math.fmod(heading +180, 260)       
        continue

    # If knocked off, mode change to slave
    slave()

def slave():
    global mode
    global reset_watchdog
    print 'SLAVE MODE'
    
    wait()

    # Start watchdog with random timeout between [25,35] interval
    reset_watchdog = 0
    watchdogThread = threading.Thread(target=watchdog,name='watchdogThread', args = (25 + rand.random()*10,))
    watchdogThread.start()

    # In slave mode, wait for radio interrupts
    while mode == 'slave':
        continue

    # If timed out, mode change to master
    master()

def watchdog(t_max):
    global mode
    global reset_watchdog
    t = 25 + rand.random()*10 # 25 to 35 sec timeout
    while t > 0:
        if reset_watchdog == 1:
            reset_watchdog = 0
            t = t_max
        t -= 1
        time.sleep(1)
    mode = 'master' # Initiate mode change

# Determine own identity - 8 digit hash
me = ''.join([rand.choice(string.ascii_letters + string.digits) for n in xrange(8)])
players = [me]

# Start compass thread at 25 Hz (DELTAT= 0.04 )
compass_enable = True
compassThread = threading.Thread(target=get_heading,name='compassThread', args = ( 0.04, ))
compassThread.start()

# Start out as slave
slave()









