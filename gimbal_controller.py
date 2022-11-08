#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import time
from threading import Thread
#import RPi.GPIO as GPIO


class Stepper:
    def __init__(self, PUL, DIR):
        self.pul_pin = PUL
        self.dir_pin = DIR
        self.step_count = 0
        self.degpstep = 0.016
        self.last_time_ms = 0
        self.pul_state = False
        self.delay = None
        self.thread_stop = False
    
    def run(self):
        self.cur_time = time.time()
        while True:
            if self.thread_stop == True:
                break
            time.sleep (0.000001)
            print (self.step_count * self.degpstep)
            if self.delay is not None and self.cur_time - self.last_time_ms > self.delay and self.pul_state == False:
                #GPIO.output(self.pul_pin, GPIO.HIGH)
                self.pul_state = True
                self.step_count += self.dir
                #print("HIGH")
            if self.delay is not None and self.cur_time - self.last_time_ms > self.delay and self.pul_state == True:
                #GPIO.output(self.pul_pin, GPIO.LOW)
                self.pul_state = False
                #print("LOW")
            self.last_time = self.cur_time

    def start_rotation(self):
        self.thread = Thread(target=self.run)
        self.thread.start()

    def stop_stepper(self):
        self.thread_stop = True

    def reset_step_count(self):
        self.step_count = 0

    def ret_angle(self):
        return self.step_count * self.degpstep

    def set_rate(self, rate_degps):
        if rate_degps > 0:
            #GPIO.output(self.dir_pin, GPIO.HIGH)
            print ("Dir HIGH")
            self.dir = 1
        else:
            #GPIO.output(self.dir_pin, GPIO.LOW)
            print ("Dir LOW")
            self.dir = -1
        if rate_degps == 0:
            self.delay = None
        else:
            self.delay = 0.016/rate_degps

if __name__ == "__main__":
    xstepper = Stepper (2, 3)
    print ("Stepper init\n")
    xstepper.start_rotation()
    time.sleep(2)
    print ("set speed\n")
    xstepper.set_rate(10)
    time.sleep(2)
    xstepper.set_rate(-50)
    time.sleep(2)
    xstepper.stop_stepper()
