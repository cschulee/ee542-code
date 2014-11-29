
# diff_drv.py
# Rover differential drive class
# Using Raspberry Pi GPIO software PWM
# Using two DRV8838 Single Brushed DC Motor Driver Carriers
#  www.pololu.com/product/2990

import RPi.GPIO as GPIO

class diff_drv:
    """Rover differential drive class for two DRV8838 Motor Drivers"""

    # Signal Mnemonics
    LOW  = 0
    HIGH = 1
    FORWARD = 1
    REVERSE = 0
    ENABLE  = 1
    DISABLE = 0

    def __init__(self,l_en_pin,l_phase_pin,l_sln_pin,r_en_pin,r_phase_pin,r_sln_pin,freq):
        
        # Set up GPIO interface
        # BOARD addressing mode - pin numbers rather than GPIO numbers
        GPIO.setmode(GPIO.BOARD)
	GPIO.setwarnings(False)
        
        # Assign arguments to local data
	self.l_en_pin     = l_en_pin    # Enable / PWM pin
        self.l_en_freq    = freq        # PWM cycle frequency	    
        self.l_phase_pin  = l_phase_pin # Phase pin
        self.l_sln_pin    = l_sln_pin   # SLEEP NOT pin
        self.r_en_pin     = r_en_pin    # Enable / PWM pin
        self.r_en_freq    = freq        # PWM cycle frequency	   
        self.r_phase_pin  = r_phase_pin # Phase pin
        self.r_sln_pin    = r_sln_pin   # !Sleep pin
        
        # Configure pins as outputs
        GPIO.setup(self.l_en_pin,    GPIO.OUT)
        GPIO.setup(self.l_phase_pin, GPIO.OUT)
        GPIO.setup(self.l_sln_pin,   GPIO.OUT)
        GPIO.setup(self.r_en_pin,    GPIO.OUT)
        GPIO.setup(self.r_phase_pin, GPIO.OUT)
        GPIO.setup(self.r_sln_pin,   GPIO.OUT)
        
        # Define/configure PWM pins
        self.l_en_pwm = GPIO.PWM(self.l_en_pin, self.l_en_freq)
        self.r_en_pwm = GPIO.PWM(self.r_en_pin, self.r_en_freq)
        
        # Set up default states - forward phase, coast drive mode
        self.l_phase = "FORWARD"
        GPIO.output(self.l_phase_pin, diff_drv.FORWARD)
        self.l_sln = "SLEEP"
        GPIO.output(self.l_sln_pin,   diff_drv.DISABLE)
        self.r_phase = "FORWARD"
        GPIO.output(self.r_phase_pin, diff_drv.FORWARD)
        self.r_sln = "SLEEP"
        GPIO.output(self.r_sln_pin,   diff_drv.DISABLE)
        self.l_en_pwm_cmd = 0
        self.r_en_pwm_cmd = 0
        
        # Start software PWMs at zero duty cycle
        self.l_en_pwm.start(0)
        self.r_en_pwm.start(0)
        
        # Enable forward and rotational speed control
        self.fwd_ctrl = diff_drv.ENABLE
        self.rot_ctrl = diff_drv.ENABLE

    def drive(self,fwd_dc, rot_dc, trim = 0):
        # Mix speed, rotation, and trim
        # Speed is positive forward
        # Rotation is positive right per right hand rule
        
        # Add trim
        self.trim = trim
        rot_dc += self.trim
        
        # Handle control modes
        if self.fwd_ctrl & self.rot_ctrl:
            left_dc  = fwd_dc - rot_dc
            right_dc = fwd_dc + rot_dc
        elif self.fwd_ctrl:
            left_dc  = fwd_dc
            right_dc = fwd_dc
        elif self.rot_ctrl:
            left_dc  = -rot_dc
            right_dc = rot_dc
        else:
            self.coast()
            return
        
        # Direction/phase discretes
        if left_dc < 0:
            self.l_phase = "REVERSE"
            GPIO.output(self.l_phase_pin, diff_drv.REVERSE)
        else:
            self.l_phase = "FORWARD"
            GPIO.output(self.l_phase_pin, diff_drv.FORWARD)

        if right_dc < 0:
            self.r_phase = "REVERSE"
            GPIO.output(self.r_phase_pin, diff_drv.REVERSE)
        else:
            self.r_phase = "FORWARD"
            GPIO.output(self.r_phase_pin, diff_drv.FORWARD)

        # Change PWM duty cycle
        self.l_en_pwm_cmd = min(100,abs(left_dc))
        self.l_en_pwm.ChangeDutyCycle(self.l_en_pwm_cmd)
        self.r_en_pwm_cmd = min(100,abs(right_dc))
        self.r_en_pwm.ChangeDutyCycle(self.r_en_pwm_cmd)

    def spd_ctrl_enable(self):
        self.fwd_ctrl_enable = diff_drv.ENABLE

    def spd_ctrl_disable(self):
        self.fwd_ctrl_enable = diff_drv.DISABLE

    def rot_ctrl_enable(self):
        self.rot_ctrl_enable = diff_drv.ENABLE

    def rot_ctrl_disable(self):
        self.rot_ctrl_enable = diff_drv.DISABLE

    def coast(self):
        self.l_sln = "SLEEP"
        GPIO.output(self.l_sln_pin,   diff_drv.DISABLE)
        self.r_sln = "SLEEP"
        GPIO.output(self.r_sln_pin,   diff_drv.DISABLE)
        self.drive(0,0,self.trim)

    def stop(self):
        self.l_en_pwm.stop()
        self.r_en_pwm.stop()

    def cleanup(self):
        GPIO.cleanup()
        
