#!/usr/bin/env python3
# Twin Otter gimbal controller code
# Tuan Luong LAGER
# 10/31/2022
# -*- coding: utf-8 -*-
import time
from threading import Thread
import RPi.GPIO as GPIO
import xmlrpc.client

## Defind Stepper motor class
class Stepper:
    def __init__(self, PUL, DIR):
        self.pul_pin = PUL
        self.dir_pin = DIR
        self.step_count = 0
        self.degpstep = 0.9/27
        self.delay_inc = 0.00001
        self.last_time = 0
        self.pul_hi_time = 0
        self.pul_state = False
        self.delay = None
        self.thread_stop = False
        GPIO.setup(self.pul_pin, GPIO.OUT, initial = GPIO.LOW)
        GPIO.setup(self.dir_pin, GPIO.OUT, initial = GPIO.LOW)
    
    def run(self):
        while True:
            if self.thread_stop == True:
                break
            self.cur_time = time.time()
            time.sleep(0.000001)
            if self.delay == 0:
                self.delay = None
            if self.delay is not None and self.cur_time - self.last_time > self.delay and self.pul_state == False:
                GPIO.output(self.pul_pin, GPIO.HIGH)
                self.pul_state = True
                self.step_count += self.dir
                self.last_time = self.cur_time
            elif self.delay is not None and self.pul_state == True and self.cur_time - self.pul_hi_time > self.delay/1.2:
                GPIO.output(self.pul_pin, GPIO.LOW)
                self.pul_state = False
                self.step_count = self.step_count
                self.pul_hi_time = self.cur_time

    def start_rotation(self):
        self.thread = Thread(target=self.run)
        self.thread.start()

    def stop_stepper(self):
        self.thread_stop = True

    def reset_step_count(self):
        self.step_count = 0

    def get_angle(self):
        return self.step_count * self.degpstep
    
    def get_step(self):
        return self.step_count

    def set_rate(self, rate_degps):
        #self.start_rotation()
        GPIO.output(self.pul_pin, GPIO.LOW)
        if rate_degps > 0:
            GPIO.output(self.dir_pin, GPIO.HIGH)
            self.dir = 1
        else:
            GPIO.output(self.dir_pin, GPIO.LOW)
            self.dir = -1
        if rate_degps == 0:
            self.delay = None
            #self.stop_stepper()
        else:
            self.delay = self.degpstep /abs(rate_degps)

## Set up interface with navigation box
HOST = '169.254.2.166'
PORT = '8000'
HostURL ='http://'+HOST+":"+PORT
backend = xmlrpc.client.ServerProxy(HostURL)
print("Server initialized\n")

pitch_deg_str = 0
roll_deg_str = 0
pitch_deg = 0
roll_deg = 0
trim_pitch = 0
trim_roll = 0
    
def checksum (sentence, chk_val):
    try:
        cksum = 0
        for i in sentence:
            cksum ^= ord(i)
            s = '{:02X}'.format(cksum)
        if s == chk_val:
            return True
        else:
            return False
    except:
        return False


def processVNINS(lineraw):
    try:
        splits = lineraw.split("*")
        chksum_val = splits[1]
        senstence = splits[0]
        header_splits = senstence.split("$")
        line = header_splits[1]
        data = line.split(",")
        if not checksum (line, chksum_val):
            return False
        global pitch_deg_str 
        pitch_deg_str = data[5]
        global roll_deg_str 
        roll_deg_str = data[6]
        return True
    except:
        return False

## Main loop
# Defind loop period
loop_time_ms = 100 #10 Hz

# Setup Raspberry PI GPIO for driving stepper motor
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)

# Set GPIO pins
mode_sw = 18
calibrate_but = 11
pitch_up_trim_but = 9
pitch_dn_trim_but = 25
roll_left_trim_but = 8
roll_right_trim_but = 7
xstepper = Stepper (2,3)
ystepper = Stepper (19,26)
#xstepper.set_rate(0);
#ystepper.set_rate(0);
xstepper.start_rotation()
ystepper.start_rotation()
print ("Stepper initialized\n")
GPIO.setup(mode_sw, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(calibrate_but, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(pitch_up_trim_but, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(pitch_dn_trim_but, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(roll_left_trim_but, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(roll_right_trim_but, GPIO.IN, pull_up_down=GPIO.PUD_UP)
print ("GPIO initialized\n")
while 1:
    try:
        
        loop_start_time_ms = time.time()
        # Get Sensor feedback
        try:
            line = backend.getVNINS()
            
        except:
            continue
        if line is None:
            continue
        if not processVNINS(line):
            continue
        pitch_deg = float(pitch_deg_str)
        roll_deg = float(roll_deg_str)
        # Calibration mode
        if GPIO.input(mode_sw):
            if not GPIO.input(pitch_up_trim_but):
                ystepper.set_rate(5)
            elif not GPIO.input(pitch_dn_trim_but):
                ystepper.set_rate(-5)
            elif not GPIO.input(roll_left_trim_but):
                xstepper.set_rate(-5)
                print("trim roll left")
            elif not GPIO.input(roll_right_trim_but):
                xstepper.set_rate(5)
                print ("trim roll right")
            elif not GPIO.input(calibrate_but):
                xstepper.reset_step_count()
                ystepper.reset_step_count()
                trim_pitch = pitch_deg
                trim_roll = roll_deg
            else:
                ystepper.set_rate(0)
                xstepper.set_rate(0)
        # Compute controller cmd
        
        # Output to motors    
        
        print (ystepper.get_angle())
        # Time delay to keeo loop at constant rate
        time.sleep((loop_time_ms - (time.time() - loop_start_time_ms))/1000) # Delay loop so we have constant loop period

    # Reset everything incase of exception    
    except Exception as e:
        print("Exception:",e)
        pitch_deg = 0
        roll_deg = 0
        backend = xmlrpc.client.ServerProxy(HostURL)   
        GPIO.clearup()
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        xstepper = Stepper (2, 3)
        ystepper = Stepper (19,26)
        xstepper.start_rotation()
        ystepper.start_rotation()    


    
