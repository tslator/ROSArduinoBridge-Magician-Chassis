/*********************************************************************
 *  ROSArduinoBridge
 
    A set of simple serial commands to control a differential drive
    robot and receive back sensor and odometry data. Default 
    configuration assumes use of an Arduino Mega + Pololu motor
    controller shield + Robogaia Mega Encoder shield.  Edit the
    readEncoder() and setMotorSpeed() wrapper functions if using 
    different motor controller or encoder method.

    Created for the Pi Robot Project: http://www.pirobot.org
    and the Home Brew Robotics Club (HBRC): http://hbrobotics.org
    
    Authors: Patrick Goebel, James Nugen

    Inspired and modeled after the ArbotiX driver by Michael Ferguson
    
    Software License Agreement (BSD License)

    Copyright (c) 2012, Patrick Goebel.
    All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions
    are met:

     * Redistributions of source code must retain the above copyright
       notice, this list of conditions and the following disclaimer.
     * Redistributions in binary form must reproduce the above
       copyright notice, this list of conditions and the following
       disclaimer in the documentation and/or other materials provided
       with the distribution.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
    "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
    LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
    FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
    COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
    INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
    BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
    CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
    LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
    ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Notes:

   This firmare has been extended to support a controller abstraction, the ROS Arduino Bridge (RAB) controller.

   Support for a pan/tilt gimble has been added along with the software servo library to replace the
   the servo library because of conflicts with the SeeedStudio motor controller.  
   There are also some command changes to support pan/tilt and a test interface.

   Otherwise, the general approach is the same as the original.

*/
#include <rab.h>
#include <DualSLD01102PMotorShield.h>
#include <DualSEN12617Encoder.h>
#include <PID_v1.h>
#include <SoftwareServo.h>

#include "commands.h"


/* Serial port baud rate */
#define BAUDRATE     57600


/* Stop the robot if it hasn't received a movement command
in this number of milliseconds */
#define AUTO_STOP_INTERVAL 3000
long lastMotorCommand = AUTO_STOP_INTERVAL;

/* Variable initialization */

// A pair of varibles to help parse serial commands (thanks Fergs)
int arg = 0;
int index = 0;

// Variable to hold an input character
char chr;

// Variable to hold the current single-character command
char cmd;

// Character arrays to hold the first and second arguments
char argv1[16];
char argv2[16];

// The arguments converted to integers
long arg1;
long arg2;
double arg3;
double arg4;

RAB rab = RAB(10, false);

SoftwareServo pan;
SoftwareServo tilt;
uint8_t pan_pin;
uint8_t tilt_pin;

uint32_t pos = 0;


/* Clear the current command parameters */
void resetCommand() {
  cmd = NULL;
  memset(argv1, 0, sizeof(argv1));
  memset(argv2, 0, sizeof(argv2));
  arg1 = 0;
  arg2 = 0;
  arg3 = 0.0;
  arg4 = 0.0;
  arg = 0;
  index = 0;
}
/* Run a command.  Commands are defined in commands.h */
int runCommand() {
  int i = 0;
  char *p = argv1;
  char *str;
  int pid_args[4];

  switch(cmd) {

  case GET_BAUDRATE:
    Serial.println(BAUDRATE);
    break;

  case ANALOG_READ:
    Serial.println(analogRead(arg1));
    break;

  case DIGITAL_READ:
    Serial.println(digitalRead(arg1));
    break;

  case ANALOG_WRITE:
    arg1 = atoi(argv1);
    arg2 = atoi(argv2);
    analogWrite(arg1, arg2);
    Serial.println("OK"); 
    break;

  case DIGITAL_WRITE:
    arg1 = atoi(argv1);
    arg2 = atoi(argv2);
    if (arg2 == 0) digitalWrite(arg1, LOW);
    else if (arg2 == 1) digitalWrite(arg1, HIGH);
    Serial.println("OK"); 
    break;
    
  case PIN_MODE:
    arg1 = atoi(argv1);
    arg2 = atoi(argv2);
    if (arg2 == 0) pinMode(arg1, INPUT);
    else if (arg2 == 1) pinMode(arg1, OUTPUT);
    Serial.println("OK");
    break;

  case PING:
    //Serial.println(Ping(arg1));
    break;

  case PAN_TILT_WRITE:
    arg1 = atoi(argv1);
    arg2 = atoi(argv2);
    pan.write(arg1);
    tilt.write(arg2);
    delay(15);
    Serial.println("OK");
    break;
    
  case SERVO_WRITE:
    Serial.println("NONE");
    break;
    
  case SERVO_READ:
    Serial.println("NONE");
    break;
  
  case PID_RATE:
    arg1 = atoi(argv1);
    rab.SetPidRate(arg1);
    Serial.println("OK");
    break;
    
  case READ_ENCODERS:
    Serial.print(rab.left_encoder_count);
    Serial.print(" ");
    Serial.println(rab.right_encoder_count);
    break;
    
   case RESET_ENCODERS:
    rab.ResetEncoderCount();
    Serial.println("OK");
    break;
    
  case MOTOR_SPEEDS:
    arg3 = atof(argv1);
    arg4 = atof(argv2);
    Serial.println(arg3);
    Serial.println(arg4);
    // Reset the auto stop timer
    lastMotorCommand = millis();
    rab.Drive(arg3, arg4);
    Serial.println("OK"); 
    break;
    
  case UPDATE_PID:
    while ((str = strtok_r(p, ":", &p)) != '\0') {
       pid_args[i] = atoi(str);
       i++;
    }
    
    //Kp = pid_args[0];
    //Kd = pid_args[1];
    //Ki = pid_args[2];
    //Ko = pid_args[3];
    Serial.println("OK");
    break;
    
  case TEST:
    arg1 = atoi(argv1);
    arg2 = atoi(argv2);
    Serial.println("Testing ...");
    switch (arg1)
    {
      case 1: // Run the Motor Test
        rab.MotorTest();
        break;
      case 2: // Run the Encoder Test
        rab.EncoderTest(arg2, arg2);
        break;
      case 3: // Run the PID Test
        rab.PidTest();
        break;
      case 4:
      {
        uint8_t ii;
        for (ii = 0; ii <= 180; ++ii)
        {
          pan.write(ii);
          tilt.write(ii);
          delay(15);
          SoftwareServo::refresh();
        }
      }
      break;
      default:
        Serial.println("Unknown Test");
        break;
    }
    Serial.println("OK");
    break;

  default:
    Serial.println("Invalid Command");
    break;
  }
}
/* Setup function--runs once at startup. */
void setup()
{
  Serial.begin(BAUDRATE);
  pan.attach(5);
  pan.setMaximumPulse(2200);
  tilt.attach(6);
  tilt.setMaximumPulse(2200);
} 
 
/* Enter the main loop.  Read and parse input from the serial port
   and run any valid commands. Run a PID calculation at the target
   interval and check for auto-stop conditions.
*/
void loop()
{ 
  while (Serial.available() > 0) {

    // Read the next character
    chr = Serial.read();
    //Serial.println(chr);

    // Terminate a command with a CR
    if (chr == 13) {
      if (arg == 1) argv1[index] = NULL;
      else if (arg == 2) argv2[index] = NULL;
      runCommand();
      resetCommand();
    }
    // Use spaces to delimit parts of the command
    else if (chr == ' ') {
      // Step through the arguments
      if (arg == 0) arg = 1;
      else if (arg == 1)  {
        argv1[index] = NULL;
        arg = 2;
        index = 0;
      }
      continue;
    }
    else {
      if (arg == 0) {
        // The first arg is the single-letter command
        cmd = chr;
      }
      else if (arg == 1) {
        // Subsequent arguments can be more than one character
        argv1[index] = chr;
        index++;
      }
      else if (arg == 2) {
        argv2[index] = chr;
        index++;
      }
    }
  }

  SoftwareServo::refresh();
  
  rab.Update();
  // Check to see if we have exceeded the auto-stop interval
  if ((millis() - lastMotorCommand) > AUTO_STOP_INTERVAL) 
  {
    rab.Drive(0.0, 0.0);
  }

}
