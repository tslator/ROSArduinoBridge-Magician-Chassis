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

from math import pi as PI, degrees, radians
import os
import time
import sys, traceback
from arduino_driver import ArduinoBase
from ros_pymata.pymatacontroller import RosArduinoBridgePyMataController, PyMata

class PymataDriver(ArduinoBase):
    def __init__(self, params, *args, **kwargs):
        super(PymataDriver, self).__init__(*args, **kwargs)    
                
        # Note: The PID Rate/Interval doesn't have to be fixed.  It can be specified in the ROS parameters and passed 
        #       into this class and onto the PyMata controller and the RAB controller (optionally)
        
        self.channel = None
        self.rab = None
        self.params = params
        
    def connect(self):
        try:
            print "Connecting to Arduino on port", self.port, "..."
            self.channel = PyMata(self.port)
            
            # Note: In order to keep the firmata firmware simple and small we create a controller to handle the integration
            #       of the motors, encoders and pid control.  Hopefully, the speed is not an issue because of the back-and-forth
            #       between the Arduino via firmata.
            self.rab = RosArduinoBridgePyMataController( self.channel, self.params )
            
            print "Connected at", self.baudrate
            print "Arduino is ready."

        except:
            print "Exception:"
            print sys.exc_info()
            print "Traceback follows:"
            traceback.print_exc(file=sys.stdout)
            print "Cannot connect to Arduino!"
            os._exit(1)

    def open(self): 
        pass

    def close(self): 
        self.rab.Close()
    
    def update_pid(self, Kp, Kd, Ki, Ko):
        ''' Set the PID parameters on the Arduino
        '''
        self.rab.UpdatePid( { 'Kp' : Kp, 'Kd' : Kd, 'Ki' : Ki, 'Ko' : Ko } )

    def get_baud(self):
        ''' Get the current baud rate on the serial port.
        '''
        return self.baudrate

    def get_encoder_counts(self):
        encoders = self.rab.ReadEncoders()
        return [ encoders['left'], encoders['right'] ]

    def reset_encoders(self):
        ''' Reset the encoder counts to 0
        '''
        self.rab.ResetEncoders()
    
    def drive(self, left, right):
        ''' Speeds are given in encoder ticks per PID interval
        '''
        # Remember: left, right are in units of ticks per PID interval
        self.rab.DriveMotors( {'left' : left, 'right' : right } )
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
        return self.channel.analog_read(pin)
    
    def analog_write(self, pin, value):
        self.channel.analog_write(pin, value)
    
    def digital_read(self, pin):
        return self.channel.digital_read(pin)
    
    def digital_write(self, pin, value):
        self.channel.digital_write(pin, value)
    
    def pin_mode(self, pin, mode):
        self.channel.set_pin_mode(pin, mode, params[pin]['type'])
    
    def servo_write(self, id, pos):
        ''' Usage: servo_write(id, pos)
            Position is given in radians and converted to degrees before sending
        '''        
        self.channel.ServoWrite(id, pos)
    
    def servo_read(self, id):
        ''' Usage: servo_read(id)
            The returned position is converted from degrees to radians
        '''        
        return radians(self.ServoRead(id))

    def ping(self, pin):
        ''' The srf05/Ping command queries an SRF05/Ping sonar sensor
            connected to the General Purpose I/O line pinId for a distance,
            and returns the range in cm.  Sonar distance resolution is integer based.
        '''
        pass
    
#    def get_maxez1(self, triggerPin, outputPin):
#        ''' The maxez1 command queries a Maxbotix MaxSonar-EZ1 sonar
#            sensor connected to the General Purpose I/O lines, triggerPin, and
#            outputPin, for a distance, and returns it in Centimeters. NOTE: MAKE
#            SURE there's nothing directly in front of the MaxSonar-EZ1 upon
#            power up, otherwise it wont range correctly for object less than 6
#            inches away! The sensor reading defaults to use English units
#            (inches). The sensor distance resolution is integer based. Also, the
#            maxsonar trigger pin is RX, and the echo pin is PW.
#        '''
#        return self.execute('z %d %d' %(triggerPin, outputPin)) 
 

""" Basic test for connectivity """
if __name__ == "__main__":
    
    if os.name == "posix":
        #portName = "/dev/ttyACM0"
        portName = "/dev/ttyS0"
    else:
        portName = "COM10" # Windows style COM port.
        
    baudRate = 57600

    params = {'pid rate' : 10}
    myArduino = PymataDriver(params=params, port=portName, baudrate=baudRate, timeout=0.5)
    myArduino.connect()
    ''' 
    print "Sleeping for 1 second..."
    time.sleep(1)   

    myArduino.digital_write(13, 1)
    time.sleep(0.5);
    myArduino.digital_write(13, 0)
    
    print "Reading on analog port 0", myArduino.analog_read(0)
    print "Reading on digital port 0", myArduino.digital_read(0)
    print "Blinking the LED 3 times"
    for i in range(3):
        myArduino.digital_write(13, 1)
        time.sleep(1.0)
        myArduino.digital_write(13, 0)
        time.sleep(1.0)
    #print "Current encoder counts", myArduino.encoders()
    
    myArduino.drive( 255, 255)
    time.sleep(1);
    '''
    print "Connection test successful.",
    
    myArduino.stop()
    pass
    
