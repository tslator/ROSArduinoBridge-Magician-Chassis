#!/usr/bin/env python

"""
    Sensor class for the arudino_python package
    
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

import roslib; roslib.load_manifest('ros_arduino_python')
import rospy
from sensor_msgs.msg import Range
from ros_arduino_msgs.msg import *

LOW = 0
HIGH = 1

INPUT = 0
OUTPUT = 1
    
class MessageType:
    ANALOG = 0
    DIGITAL = 1
    RANGE = 2
    FLOAT = 3
    INT = 4
    BOOL = 5
    ARRAY = 6
    
class Sensor(object):
    def __init__(self, controller, name, pin, rate, frame_id, direction="input", **kwargs):
        self.controller = controller
        self.name = name
        self.pin = pin
        self.rate = rate
        self.direction = direction

        self.frame_id = frame_id
        self.value = None
        
        self.t_delta = rospy.Duration(1.0 / self.rate)
        self.t_next = rospy.Time.now() + self.t_delta
    
    def poll(self):
        now = rospy.Time.now()
        if now > self.t_next:
            if self.direction == "input":
                try:
                    self.value = self.read_value()
                except:
                    return
            else:
                try:
                    self.ack = self.write_value()
                except:
                    return          
    
            # For range sensors, assign the value to the range message field
            if self.message_type == MessageType.RANGE:
                self.msg.range = self.value
            else:
                self.msg.value = self.value

            # Add a timestamp and publish the message
            self.msg.header.stamp = rospy.Time.now()
            self.pub.publish(self.msg)
            
            self.t_next = now + self.t_delta
            
class SensorArray(object):
    def __init__(self, rate, *args, **kwargs):
        
        self.message_type = MessageType.ARRAY
        self.sensors = []
        self.value = []
        self.rate = rate
        self.t_delta = rospy.Duration(1.0 / self.rate)
        self.t_next = rospy.Time.now() + self.t_delta

    def poll(self):
        now = rospy.Time.now()
        if now > self.t_next:
            for i in range(len(self.sensors)):
                self.sensors[i].poll()
                self.value[i] = self.sensors[i].read_value()

            self.msg.value = self.value
            self.msg.header.stamp = rospy.Time.now()
            self.pub.publish(self.msg)

            self.t_next = now + self.t_delta
        
    
class AnalogSensor(Sensor):
    def __init__(self, *args, **kwargs):
        super(AnalogSensor, self).__init__(*args, **kwargs)
                
        self.message_type = MessageType.ANALOG
        
        self.msg = Analog()
        self.msg.header.frame_id = self.frame_id
        
        self.pub = rospy.Publisher("~sensor/" + self.name, Analog, queue_size=10)
        
        if self.direction == "output":
            self.controller.pin_mode(self.pin, OUTPUT)
        else:
            self.controller.pin_mode(self.pin, INPUT)

        self.value = LOW
        
    def read_value(self):
        return self.controller.analog_read(self.pin)
    
    def write_value(self, value):
        return self.controller.analog_write(self.pin, value)
    
class AnalogFloatSensor(Sensor):
    def __init__(self, *args, **kwargs):
        super(AnalogFloatSensor, self).__init__(*args, **kwargs)
                
        self.message_type = MessageType.ANALOG
        
        self.msg = AnalogFloat()
        self.msg.header.frame_id = self.frame_id
        
        self.pub = rospy.Publisher("~sensor/" + self.name, AnalogFloat, queue_size=10)
        
        if self.direction == "output":
            self.controller.pin_mode(self.pin, OUTPUT)
        else:
            self.controller.pin_mode(self.pin, INPUT)

        self.value = LOW
        
    def read_value(self):
        return self.controller.analog_read(self.pin)
    
    def write_value(self, value):
        return self.controller.analog_write(self.pin, value)
    
        
class DigitalSensor(Sensor):
    def __init__(self, pullup, *args, **kwargs):
        super(DigitalSensor, self).__init__(*args, **kwargs)
        
        self.message_type = MessageType.BOOL
        
        self.msg = Digital()
        self.msg.header.frame_id = self.frame_id
        
        self.pub = rospy.Publisher("~sensor/" + self.name, Digital, queue_size=10)
       
 
        if self.direction == "output":
            self.controller.pin_mode(self.pin, OUTPUT)
        else:
            self.controller.pin_mode(self.pin, INPUT)
            if pullup:
                self.controller.digital_write(self.pin, HIGH)

        self.value = LOW
        
    def read_value(self):
        return self.controller.digital_read(self.pin)
    
    def write_value(self):
        # Alternate HIGH/LOW when writing at a fixed rate
        self.value = not self.value
        return self.controller.digital_write(self.pin, self.value)
    

class DigitalSensorArray(SensorArray):
    def __init__(self, controller, name, pins, pullup, frame_id, *args, **kwargs):
        super(DigitalSensorArray, self).__init__(*args, **kwargs)

        self.msg = DigitalArray()
        self.frame_id = frame_id
        self.msg.header.frame_id = self.frame_id
        self.name = name
        self.pins = pins

        self.pub = rospy.Publisher("~sensor/" + self.name, DigitalArray, queue_size=10)
        for i in range(len(self.pins)):
            self.sensors.append(DigitalSensor(pullup,
                                              controller, 
                                              self.name + "_" + str(i), 
                                              self.pins[i], 
                                              kwargs['rate'],
                                              self.frame_id,
                                              direction=kwargs['direction']))
            self.value.append(LOW)

    def read_value(self):
        return self.value
    
    
class RangeSensor(Sensor):
    def __init__(self, *args, **kwargs):
        super(RangeSensor, self).__init__(*args, **kwargs)
        
        self.message_type = MessageType.RANGE
        
        self.msg = Range()
        self.msg.header.frame_id = self.frame_id
        
        self.pub = rospy.Publisher("~sensor/" + self.name, Range)
        
    def read_value(self):
        self.msg.header.stamp = rospy.Time.now()

        
class SonarSensor(RangeSensor):
    def __init__(self, *args, **kwargs):
        super(SonarSensor, self).__init__(*args, **kwargs)
        
        self.msg.radiation_type = Range.ULTRASOUND
        
        
class IRSensor(RangeSensor):
    def __init__(self, *args, **kwargs):
        super(IRSensor, self).__init__(*args, **kwargs)
        
        self.msg.radiation_type = Range.INFRARED
        
class Ping(SonarSensor):
    def __init__(self,*args, **kwargs):
        super(Ping, self).__init__(*args, **kwargs)
                
        self.msg.field_of_view = 0.785398163
        self.msg.min_range = 0.02
        self.msg.max_range = 3.0
        
    def read_value(self):
        # The Arduino Ping code returns the distance in centimeters
        cm = self.controller.ping(self.pin)
        
        # Convert it to meters for ROS
        distance = cm / 100.0
        
        return distance
    
        
class GP2D12(IRSensor):
    def __init__(self, *args, **kwargs):
        super(GP2D12, self).__init__(*args, **kwargs)
        
        self.msg.field_of_view = 0.001
        self.msg.min_range = 0.10
        self.msg.max_range = 0.80
        
    def read_value(self):
        value = self.controller.analog_read(self.pin)
        
        if value <= 3.0:
            return self.msg.max_range
        
        try:
            distance = (6787.0 / (float(value) - 3.0)) - 4.0
        except:
            return self.msg.max_range
            
        # Convert to meters
        distance /= 100.0
        
        # If we get a spurious reading, set it to the max_range
        if distance > self.msg.max_range: distance = self.msg.max_range
        if distance < self.msg.min_range: distance = self.msg.max_range
        
        return distance
    
class PololuMotorCurrent(AnalogFloatSensor):
    def __init__(self, *args, **kwargs):
        super(PololuMotorCurrent, self).__init__(*args, **kwargs)
        
    def read_value(self):
        # From the Pololu source code
        milliamps = self.controller.analog_read(self.pin) * 34
        return milliamps / 1000.0
    
class PhidgetsVoltage(AnalogFloatSensor):
    def __init__(self, *args, **kwargs):
        super(PhidgetsVoltage, self).__init__(*args, **kwargs)
        
    def read_value(self):
        # From the Phidgets documentation
        voltage = 0.06 * (self.controller.analog_read(self.pin) - 500.)
        return voltage
    
class PhidgetsCurrent(AnalogFloatSensor):
    def __init__(self, *args, **kwargs):
        super(PhidgetsCurrent, self).__init__(*args, **kwargs)
        
    def read_value(self):
        # From the Phidgets documentation for the 20 amp DC sensor
        current = 0.05 * (self.controller.analog_read(self.pin) - 500.)
        return current
    
class MaxEZ1Sensor(SonarSensor):
    def __init__(self, *args, **kwargs):
        super(MaxEZ1Sensor, self).__init__(*args, **kwargs)
        
        self.trigger_pin = kwargs['trigger_pin']
        self.output_pin = kwargs['output_pin']
        
        self.msg.field_of_view = 0.785398163
        self.msg.min_range = 0.02
        self.msg.max_range = 3.0
        
    def read_value(self):
        return self.controller.get_MaxEZ1(self.trigger_pin, self.output_pin)


class LineSensor(DigitalSensorArray):
    def __init__(self, *args, **kwargs):
        super(LineSensor, self).__init__(*args, **kwargs)
        
        pass
        
        
if __name__ == '__main__':
    myController = Controller()
    mySensor = SonarSensor(myController, "My Sonar", type=Type.PING, pin=0, rate=10)
            
