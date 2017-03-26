# ROSArduinoBridge-Magician-Chassis
A port of the ROS Arduino Bridge (https://github.com/hbrobotics/ros_arduino_bridge.git) using the Sparkfun Magician Chassis.

This project is a port of the ROS Arduino Bridge for the following hardware:
* Sparkfun Magician Chassis (https://www.sparkfun.com/products/retired/10825)
* Sparkfun Dual SEN12617 Encoder (https://www.sparkfun.com/products/12617)
* Raspberry Pi 2
* Alamode Arduino Raspberry Pi Shield (http://wyolum.com/projects/alamode/)
* Sparkfun Pan/Tilt Bracket (https://www.sparkfun.com/products/10335) w/ Servos
* Raspberry Pi Camera (TODO)
* Line sensor (TODO)

The hardware for this project essentially started with what is offerred on the Sparkfun website using a Raspberry Pi and the Dagu motor driver (DG-0813).  Initially, the fine software provided by Dawn Robotics as seen here (https://bitbucket.org/DawnRobotics/raspberry_pi_camera_bot) was used and the general architecture is the same, i.e., Raspberry Pi to host the web server and receive commands from a web page and Arduino to receive sensor input and control the pan/tilt servos and drive motors.

The goal originally was to use ROS but the Raspberry Pi's at that time weren't powerful enough.  When the Raspberry Pi 2 became available, using ROS was viable and this project began.  Using the ROS Arduino Bridge as a baseline, extensions were added to support the Sparkfun hardware.  The Dagu motor driver was replaced with the Alamode and Seeed Studio motor shields for a more compact design.  The Arduino firmware was abstracted to create a single controller that supports a differential drive with encoders and PID control.

With ROS came the ability to modularize the project and take advantage of standard modules such as the ROS Bridge which provides a Javascript library for publishing and subscribing to topics.  The web page has been reworked to use the ROS Javascript library.

Future additions to this project will include:
* using the ROS sensor/camera message to send images from the Raspberry Pi camera to the web page
* a line sensor for line following.  Note: the ROS pub/sub model should make this an especially elegant solution
* using OpenCV to perform image processing such as facial recognition, object tracking etc.

# Pictures
![alt text](https:/github.com/tslator/ROSArduinoBridge-Magician-Chassis/raw/master/src/images/001.jpg)
