#!/usr/bin/env python3
# Twin Otter gimbal controller code
# Tuan Luong LAGER
# 10/31/2022
# -*- coding: utf-8 -*-
import time
import serial
from threading import Thread
import RPi.GPIO as GPIO
import xmlrpc.client

## Defind Stepper motor class
class Stepper:
    def __init__(self, PUL, DIR):
        self.pul_pin = PUL
        self.dir_pin = DIR
        self.step_count = 0
        self.cur_ang_deg = 0
        self.degpstep = 0.9/27
        self.last_time = 0
        self.pul_hi_time = 0
        self.pul_state = False
        self.delay = None
        self.thread_stop = False
        self.is_reversed = False
        self.angle_lim = 9
        GPIO.setup(self.pul_pin, GPIO.OUT, initial = GPIO.LOW)
        GPIO.setup(self.dir_pin, GPIO.OUT, initial = GPIO.LOW)
        
    def set_angle_limit(self, angle_lim):
        self.angle_lim = angle_lim
        
    def set_angle_pos(self,cur_ang):
        self.cur_ang_deg = cur_ang
    
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
    
    def reverse_dir(self, is_reversed):
        self.is_reversed = is_reversed

    def set_rate(self, rate_degps):
        GPIO.output(self.pul_pin, GPIO.LOW)
        if rate_degps > 0:
            #print (self.cur_ang_deg)
            if self.cur_ang_deg > self.angle_lim:
                #print ("limit_reached")
                self.dir = 0
                self.delay = None
            else:
                if self.is_reversed:
                    GPIO.output(self.dir_pin, GPIO.HIGH)
                    self.dir = 1
                else:
                    GPIO.output(self.dir_pin, GPIO.LOW)
                    self.dir = 1
                self.delay = self.degpstep /abs(rate_degps)
        elif rate_degps < 0:
            if self.cur_ang_deg < -self.angle_lim:
                self.dir = 0
                self.delay = None
            else:
                if self.is_reversed:
                    GPIO.output(self.dir_pin, GPIO.LOW)
                    self.dir = -1
                else:
                    GPIO.output(self.dir_pin, GPIO.HIGH)
                    self.dir = -1
                self.delay = self.degpstep / abs(rate_degps)
        elif rate_degps == 0:
            self.delay = None

## Set up interface with navigation box
HOST = '169.254.2.166'
PORT = '8000'
HostURL ='http://'+HOST+":"+PORT
backend = xmlrpc.client.ServerProxy(HostURL)
print("Server initialized\n")

## Setup serial port for rotary encoder
rotary_port = '/dev/ttyACM0'
rotary_baud = 115200

# Airplane angle from horizon (theta_plane_in_inertial)
plane_pitch_deg_str = "0"
plane_roll_deg_str = "0"
plane_pitch_deg = 0
plane_roll_deg = 0

# Angle between plane and gimbal
trim_plane_gimbal_pitch = 0
trim_plane_gimbal_roll = 0
with open ('/home/pi/calib.txt', mode='r') as f:
    trim_str = f.readline().rstrip()
    trim_splits = trim_str.split(",")
    trim_plane_gimbal_roll = float(trim_splits[0])
    trim_plane_gimbal_pitch = float(trim_splits[1])

# Angle between antenna to gimbal 
rotary_pitch = 0
rotary_roll = 0
rotary_pitch_center = 0
rotary_roll_center = 0
with open ('/home/pi/rotary_center.txt', mode='r') as f:
    trim_str = f.readline().rstrip()
    trim_splits = trim_str.split(",")
    rotary_roll_center = float(trim_splits[0])
    rotary_pitch_center = float(trim_splits[1])

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
        global plane_pitch_deg_str 
        plane_pitch_deg_str = data[5]
        global plane_roll_deg_str 
        plane_roll_deg_str = data[6]
        return True
    except:
        return False

## Main loop
# Defind loop period
loop_time_s = 0.05  #20 Hz

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
xstepper.set_angle_limit(15)
ystepper.set_angle_limit(15)

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
        loop_start_time_s = time.time()
        # Get Sensor feedback
        try:
            line = backend.getVNINS()
        except:
            #print("Can't get backend")
            continue
        if line is None:
            #print("No line retrieved")
            continue
        if not processVNINS(line):
            #print("Invalid line")
            continue
        plane_pitch_deg = float(plane_pitch_deg_str)
        plane_roll_deg = float(plane_roll_deg_str)
        
        # Read rotary angle
        try:
            with serial.Serial(rotary_port, rotary_baud, timeout = 5) as ser:
                ser.reset_input_buffer()
                rotary_buf = ser.readline()
                rotary_str = rotary_buf.decode()
                rotary_splits = rotary_str.split(",")
                rotary_roll = float(rotary_splits[1])
                rotary_pitch = float(rotary_splits[2])
        except:
            #print ("No rotary value")
            continue
        ystepper.set_angle_pos(rotary_pitch - rotary_pitch_center)
        xstepper.set_angle_pos(rotary_roll - rotary_roll_center)
        # Calibration mode
        if GPIO.input(mode_sw):
            if not GPIO.input(pitch_up_trim_but):
                ystepper.set_rate(1)
            elif not GPIO.input(pitch_dn_trim_but):
                ystepper.set_rate(-1)
            elif not GPIO.input(roll_left_trim_but):
                xstepper.set_rate(-1)
            elif not GPIO.input(roll_right_trim_but):
                xstepper.set_rate(1)
            elif not GPIO.input(calibrate_but):
                # TODO: Maybe calculate conversion between plane and angle once
                trim_plane_gimbal_pitch = - plane_pitch_deg
                trim_plane_gimbal_roll = - plane_roll_deg 
                with open ('/home/pi/calib.txt', mode='w') as f:
                    f.write(str(trim_plane_gimbal_roll))
                    f.write(",")
                    f.write(str(trim_plane_gimbal_pitch))
                rotary_roll_center = rotary_roll
                rotary_pitch_center = rotary_pitch
                with open ('/home/pi/rotary_center.txt', mode='w') as f:
                    f.write(str(rotary_roll_center))
                    f.write(",")
                    f.write(str(rotary_pitch_center))
                    
            else:
                ystepper.set_rate(0)
                xstepper.set_rate(0)
                
        # Compute controller cmd if not in calibration mode
        else:
            inertial_pitch = plane_pitch_deg + trim_plane_gimbal_pitch + rotary_pitch - rotary_pitch_center
            inertial_roll = plane_roll_deg + trim_plane_gimbal_roll + rotary_roll - rotary_roll_center
            print(inertial_pitch)
            try:
                backend.record_gimbal_angles(inertial_pitch, inertial_roll)
            except Exception as e:
                print (e)
                continue
            pitch_error = 0 - inertial_pitch
            roll_error = 0 - inertial_roll
            pitch_cmd = 12 * pitch_error
            roll_cmd = 12  * roll_error
            if pitch_cmd > 30:
                pitch_cmd = 30
            elif pitch_cmd < -30:
                pitch_cmd = -30
            if roll_cmd > 30:
                roll_cmd = 30
            elif roll_cmd < -30:
                roll_cmd = -30
            # Output to motor
            ystepper.set_rate(int(pitch_cmd))
            xstepper.set_rate(int(roll_cmd))
        #print(rotary_roll)
        # Time delay to keeo loop at constant rate
        try:
            time.sleep(loop_time_s - (time.time() - loop_start_time_s)) # Delay loop so we have constant loop period
        except:
            continue

    # Reset everything incase of exception    
    except Exception as e:
        print (e)
        backend = xmlrpc.client.ServerProxy(HostURL)   
        GPIO.cleanup()
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        xstepper = Stepper (2,3)
        ystepper = Stepper (19,26)
        xstepper.start_rotation
        ystepper.start_rotation
        ystepper.set_rate(0)
        xstepper.set_rate(0)
        GPIO.setup(mode_sw, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(calibrate_but, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(pitch_up_trim_but, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(pitch_dn_trim_but, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(roll_left_trim_but, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(roll_right_trim_but, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        # Airplane angle from horizon (theta_plane_in_inertial)
        plane_pitch_deg_str = "0"
        plane_roll_deg_str = "0"
        plane_pitch_deg = 0
        plane_roll_deg = 0

        # Angle between plane and gimbal
        trim_plane_gimbal_pitch = 0
        trim_plane_gimbal_roll = 0
        with open ('/home/pi/calib.txt', mode='r') as f:
            trim_str = f.readline().rstrip()
            trim_splits = trim_str.split(",")
            trim_plane_gimbal_roll = float(trim_splits[0])
            trim_plane_gimbal_pitch = float(trim_splits[1])

        # Angle between antenna to gimbal 
        rotary_pitch = 0
        rotary_roll = 0
        rotary_pitch_center = 0
        rotary_roll_center = 0
        with open ('/home/pi/rotary_center.txt', mode='r') as f:
            trim_str = f.readline().rstrip()
            trim_splits = trim_str.split(",")
            rotary_roll_center = float(trim_splits[0])
            rotary_pitch_center = float(trim_splits[1])
        

    

