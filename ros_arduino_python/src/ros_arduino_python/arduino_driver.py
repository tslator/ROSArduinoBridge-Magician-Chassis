#!/usr/bin/env python

"""
    A Python driver for the Arduino microcontroller running the
    ROSArduinoBridge firmware.
    
    Created for the Pi Robot Project: http://www.pirobot.org
    Copyright (c) 2012 Patrick Goebel.  All rights reserved.

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.
    
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details at:
    
    http://www.gnu.org/licenses/gpl.html

"""

import thread
from math import pi as PI, degrees, radians
import os
import time
import sys, traceback
from serial.serialutil import SerialException
from serial import Serial


class ArduinoBase(object):
    SERVO_MAX = 180
    SERVO_MIN = 0

    ''' Configuration Parameters
    '''    
    def __init__(self, port="/dev/ttyUSB0", baudrate=57600, timeout=0.5, *args,     **kwargs):
        
        self.PID_RATE = 30 # Do not change this!  It is a fixed property of the Arduino PID controller.
        self.PID_INTERVAL = 1000 / self.PID_RATE
        
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.writeTimeout = timeout
        
    def connect(self):
        pass

    def update_pid(self, Kp, Kd, Ki, Ko):
        ''' Set the PID parameters on the Arduino
        '''
        pass                          

    def get_baud(self):
        ''' Get the current baud rate on the serial port.
        '''
        return int(self.execute('b'));

    def get_encoder_counts(self):
        ''' Get the encoder counts
        '''
        pass

    def reset_encoders(self):
        ''' Reset the encoder counts to 0
        '''
        pass
    
    def drive(self, right, left):
        ''' Speeds are given in encoder ticks per PID interval
        '''
        pass
    
    def drive_m_per_s(self, right, left):
        ''' Set the motor speeds in meters per second.
        '''
        left_revs_per_second = float(left) / (self.wheel_diameter * PI)
        right_revs_per_second = float(right) / (self.wheel_diameter * PI)

        left_ticks_per_loop = int(left_revs_per_second * self.encoder_resolution * self.PID_INTERVAL * self.gear_reduction)
        right_ticks_per_loop  = int(right_revs_per_second * self.encoder_resolution * self.PID_INTERVAL * self.gear_reduction)

        self.drive(right_ticks_per_loop , left_ticks_per_loop )
        
    def stop(self):
        ''' Stop both motors.
        '''
        self.drive(0, 0)
            
    def analog_read(self, pin):
        pass
    
    def analog_write(self, pin, value):
        pass
    
    def digital_read(self, pin):
        pass
    
    def digital_write(self, pin, value):
        pass
    
    def pin_mode(self, pin, mode):
        pass

    def servo_write(self, id, pos):
        ''' Usage: servo_write(id, pos)
            Position is given in radians and converted to degrees before sending
        '''        
        pass
    
    def servo_read(self, id):
        ''' Usage: servo_read(id)
            The returned position is converted from degrees to radians
        '''        
        pass

    def ping(self, pin):
        ''' The srf05/Ping command queries an SRF05/Ping sonar sensor
            connected to the General Purpose I/O line pinId for a distance,
            and returns the range in cm.  Sonar distance resolution is integer based.
        '''
        pass