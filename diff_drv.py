
# diff_drv.py
# Rover differential drive class
# Using Raspberry Pi GPIO software PWM
# Using two DRV8838 Single Brushed DC Motor Driver Carriers
#  www.pololu.com/product/2990

import RPi.GPIO as GPIO

class diff_drv:

    # Signal Mnemonics
    LOW  = 0
    HIGH = 1
    FORWARD = 1
    REVERSE = 0
    ENABLE  = 1
    DISABLE = 0

    def __init__(self,PWM_L,PWM_R,PH_L,PH_R,EN_L,EN_R,FREQ):
	GPIO.setmode(GPIO.BCM)
	GPIO.setwarnings(False)
	self.pwm_l = PWM_L
	self.pwm_r = PWM_R
	self.ph_l  = PH_L
	self.ph_r   = PH_R
	self.en_l  = EN_L
	self.en_r  = EN_R
        GPIO.setup(self.pwm_l, GPIO.OUT)
        GPIO.setup(self.pwm_r, GPIO.OUT)
        GPIO.setup(self.ph_l, GPIO.OUT)
        GPIO.setup(self.ph_r, GPIO.OUT)
        GPIO.setup(self.en_l, GPIO.OUT)
        GPIO.setup(self.en_r, GPIO.OUT)
        self.left  = GPIO.PWM(self.pwm_l,FREQ)
        self.right = GPIO.PWM(self.pwm_r,FREQ)
        GPIO.output(self.ph_l,diff_drv.FORWARD)
        GPIO.output(self.ph_r,diff_drv.FORWARD)
        GPIO.output(self.en_l,diff_drv.ENABLE)
        GPIO.output(self.en_r,diff_drv.ENABLE)
        self.left.start(0)
        self.right.start(0)
        self.spd_ctrl = diff_drv.ENABLE
        self.rot_ctrl = diff_drv.ENABLE
        return

    def drive(self,spd_dc,rot_dc):
        # Mix speed and rotation
        # Speed is positive forward
        # Rotation is positive right per right hand rule
        left_dc  = spd_dc - rot_dc
        right_dc = spd_dc + rot_dc

        if left_dc < 0:
            GPIO.output(self.ph_l,diff_drv.REVERSE)
        else:
            GPIO.output(self.ph_l,diff_drv.FORWARD)

        if right_dc < 0:
            GPIO.output(self.ph_r,diff_drv.REVERSE)
        else:
            GPIO.output(self.ph_r,diff_drv.FORWARD)

        self.left.ChangeDutyCycle(min(100,abs(left_dc)))
        self.right.ChangeDutyCycle(min(100,abs(right_dc)))
        return

    def spd_ctrl_enable(self):
        self.spd_ctrl = diff_drv.ENABLE
        return

    def spd_ctrl_disable(self):
        self.spd_ctrl = diff_drv.DISABLE
        return

    def rot_ctrl_enable(self):
        self.rot_ctrl = diff_drv.ENABLE
        return

    def rot_ctrl_disable(self):
        self.rot_ctrl = diff_drv.DISABLE
        return

    def coast(self):
        self.drive(0,0)
        return

    def stop(self):
        self.left.stop()
        self.right.stop()
        
