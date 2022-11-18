#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import time
from threading import Thread
import RPi.GPIO as GPIO


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
        GPIO.output(self.pul_pin, GPIO.LOW)
        if rate_degps > 0:
            GPIO.output(self.dir_pin, GPIO.HIGH)
            self.dir = 1
        else:
            GPIO.output(self.dir_pin, GPIO.LOW)
            self.dir = -1
        if rate_degps == 0:
            self.delay = None
        else:
            self.delay = self.degpstep /abs(rate_degps)

if __name__ == "__main__":
    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BCM)
    xstepper = Stepper (2, 3)
    ystepper = Stepper (19,26)
    print ("Stepper init\n")
    xstepper.start_rotation()
    ystepper.start_rotation()
    xstepper.set_rate(5)
    ystepper.set_rate(15)
        


    
