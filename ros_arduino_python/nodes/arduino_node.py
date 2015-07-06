#!/usr/bin/env python

"""
    A ROS Node for the Arduino microcontroller
    
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

import rospy
from ros_arduino_python.arduino_sensors import *
from ros_arduino_msgs.srv import *
from ros_arduino_python.base_controller import BaseController
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from ros_arduino_msgs.msg import PanTilt
import os, time
import thread

class ArduinoROS():
    def __init__(self):
        rospy.init_node('Arduino', log_level=rospy.DEBUG)
                
        # Cleanup when termniating the node
        rospy.on_shutdown(self.shutdown)
        
        self.port = rospy.get_param("~port", "/dev/ttyACM0")
        self.baud = int(rospy.get_param("~baud", 57600))
        self.timeout = rospy.get_param("~timeout", 0.5)
        self.base_frame = rospy.get_param("~base_frame", 'base_link')

        self.l_scale = rospy.get_param("~linear_scale", 10)
        self.a_scale = rospy.get_param("~angular_scale", 90)

        # Overall loop rate: should be faster than fastest sensor rate
        self.rate = int(rospy.get_param("~rate", 50))
        r = rospy.Rate(self.rate)

        # Rate at which summary SensorState message is published. Individual sensors publish
        # at their own rates.        
        self.sensorstate_rate = int(rospy.get_param("~sensorstate_rate", 10))
        
        self.use_base_controller = rospy.get_param("~use_base_controller", False)
        
        # Set up the time for publishing the next SensorState message
        now = rospy.Time.now()
        self.t_delta_sensors = rospy.Duration(1.0 / self.sensorstate_rate)
        self.t_next_sensors = now + self.t_delta_sensors
        
        rospy.Subscriber("html_joy_stick", Joy, self.JoystickCallback)
        
        # Initialize a Twist message
        self.cmd_vel = Twist()
  
        # A cmd_vel publisher so we can stop the robot when shutting down
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
       
        # A pan_tilt publisher so we can control the position of the pan/tilt gimble and center it on shutdown
        self.pan_tilt_pub = rospy.Publisher('pan_tilt', PanTilt, queue_size=10)
 
        # The SensorState publisher periodically publishes the values of all sensors on
        # a single topic.
        self.sensorStatePub = rospy.Publisher('~sensor_state', SensorState, queue_size=10)
        
        # A service to position a PWM servo
        rospy.Service('~servo_write', ServoWrite, self.ServoWriteHandler)
        
        # A service to read the position of a PWM servo
        rospy.Service('~servo_read', ServoRead, self.ServoReadHandler)
        
        # A service to turn set the direction of a digital pin (0 = input, 1 = output)
        rospy.Service('~digital_set_direction', DigitalSetDirection, self.DigitalSetDirectionHandler)
        
        # A service to turn a digital sensor on or off
        rospy.Service('~digital_write', DigitalWrite, self.DigitalWriteHandler)
       
        # A service to set pwm values for the pins
        rospy.Service('~analog_write', AnalogWrite, self.AnalogWriteHandler)

        # Initialize the controller
        self.controller = None
        driver = rospy.get_param("~arduino_driver", "rab_driver")
        params = { 'pid rate' : rospy.get_param("~base_controller_rate", 10) }
        if driver == "rab_driver":
            from ros_arduino_python.rab_driver import RABDriver
            self.controller = RABDriver(params=params, port=self.port, baud=self.baud, timeout=self.timeout)
        elif driver == "pymata_driver":
            from ros_arduino_python.pymata_driver import PymataDriver
            self.controller = PymataDriver(params=params, port=self.port, baud=self.baud, timeout=self.timeout)

        else:
            rospy.loginfo("Unknown Arduino driver")
            return
        
        # Make the connection
        self.controller.connect()
        
        rospy.loginfo("Connected to Arduino on port " + self.port + " at " + str(self.baud) + " baud")
     
        # Reserve a thread lock
        mutex = thread.allocate_lock()

        # Initialize any sensors
        self.mySensors = list()
        
        sensor_params = rospy.get_param("~sensors", dict({}))
        
        for name, params in sensor_params.iteritems():
            # Set the direction to input if not specified
            try:
                params['direction']
            except:
                params['direction'] = 'input'
                
            if params['type'] == "Ping":
                sensor = Ping(self.controller, name, params['pin'], params['rate'], self.base_frame)
            elif params['type'] == "GP2D12":
                sensor = GP2D12(self.controller, name, params['pin'], params['rate'], self.base_frame)
            elif params['type'] == 'Digital':
                sensor = DigitalSensor(self.controller, name, params['pin'], params['rate'], self.base_frame, direction=params['direction'])
            elif params['type'] == 'Analog':
                sensor = AnalogSensor(self.controller, name, params['pin'], params['rate'], self.base_frame, direction=params['direction'])
            elif params['type'] == 'PololuMotorCurrent':
                sensor = PololuMotorCurrent(self.controller, name, params['pin'], params['rate'], self.base_frame)
            elif params['type'] == 'PhidgetsVoltage':
                sensor = PhidgetsVoltage(self.controller, name, params['pin'], params['rate'], self.base_frame)
            elif params['type'] == 'PhidgetsCurrent':
                sensor = PhidgetsCurrent(self.controller, name, params['pin'], params['rate'], self.base_frame)
            elif params['type'] == 'DigitalArray':
                sensor = LineSensor(self.controller, name, params['pins'], params['pullup'], self.base_frame, rate=params['rate'], direction=params['direction'])
                
#                if params['type'] == "MaxEZ1":
#                    self.sensors[len(self.sensors)]['trigger_pin'] = params['trigger_pin']
#                    self.sensors[len(self.sensors)]['output_pin'] = params['output_pin']

            self.mySensors.append(sensor)
            rospy.loginfo(name + " " + str(params))
              
        # Initialize the base controller if used
        if self.use_base_controller:
            self.myBaseController = BaseController(self.controller, self.base_frame)
    
        # Start polling the sensors and base controller
        while not rospy.is_shutdown():
            for sensor in self.mySensors:
                mutex.acquire()
                sensor.poll()
                mutex.release()
                    
            if self.use_base_controller:
                mutex.acquire()
                self.myBaseController.poll()
                mutex.release()
            
            # Publish all sensor values on a single topic for convenience
            now = rospy.Time.now()
            
            if now > self.t_next_sensors:
                msg = SensorState()
                msg.header.frame_id = self.base_frame
                msg.header.stamp = now
                for i in range(len(self.mySensors)):
                    msg.name.append(self.mySensors[i].name)
                    msg.value.append(self.mySensors[i].value)
                try:
                    self.sensorStatePub.publish(msg)
                except:
                    pass
                
                self.t_next_sensors = now + self.t_delta_sensors
            
            r.sleep()
    
    # Service callback functions
    def ServoWriteHandler(self, req):
        self.controller.servo_write(req.id, req.value)
        return ServoWriteResponse()
    
    def ServoReadHandler(self, req):
        self.controller.servo_read(req.id)
        return ServoReadResponse()
    
    def DigitalSetDirectionHandler(self, req):
        self.controller.pin_mode(req.pin, req.direction)
        return DigitalSetDirectionResponse()
    
    def DigitalWriteHandler(self, req):
        self.controller.digital_write(req.pin, req.value)
        return DigitalWriteResponse()
              
    def AnalogWriteHandler(self, req):
        self.controller.analog_write(req.pin, req.value)
        return AnalogWriteResponse()
 
    def JoystickCallback(self, data):
        #rospy.loginfo("Received joystick msg: " + str(data))
        
        # axes 0 and 1 are assigned to the drive train
        # axes 2 and 3 are assigned to the pan/tilt
        
        # Handle motor drive/turning
        twist = Twist()
        twist.linear.x = self.l_scale*data.axes[0] # forward/backward speed
        twist.linear.y = 0
        twist.linear.z = 0        
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = self.a_scale*data.axes[1] # turning speed
        self.cmd_vel_pub.publish(twist)

        # Handle pan/tilt
        # The value received on axes 3 and 4 ranges from -1.0 to 1.0
        pan_tilt = PanTilt()
        pan_tilt.pan = data.axes[2]
        pan_tilt.tilt = data.axes[3]
        #rospy.loginfo("Publishing: pan %f tilt %f" % (pan_tilt.pan, pan_tilt.tilt))
        self.pan_tilt_pub.publish(pan_tilt) 
        
    
    def shutdown(self):
        # Stop the robot
        try:
            rospy.loginfo("Stopping the robot...")
            self.cmd_vel_pub.Publish(Twist())
            rospy.sleep(2)
        except:
            pass
        rospy.loginfo("Shutting down Arduino Node...")
        
if __name__ == '__main__':
    myArduino = ArduinoROS()
