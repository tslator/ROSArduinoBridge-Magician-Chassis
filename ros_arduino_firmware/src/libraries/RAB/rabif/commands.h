/* Define single-letter commands that will be sent by the PC over the
   serial link.
*/

#ifndef COMMANDS_H
#define COMMANDS_H

#define ANALOG_READ    'a'
#define GET_BAUDRATE   'b'
#define PIN_MODE       'c'
#define DIGITAL_READ   'd'
#define READ_ENCODERS  'e'
#define PID_RATE       'i'
#define MOTOR_SPEEDS   'm'
#define PING           'g'
#define RESET_ENCODERS 'r'
#define PAN_TILT_WRITE 'p'
#define SERVO_WRITE    's'
#define SERVO_READ     't'
#define UPDATE_PID     'u'
#define DIGITAL_WRITE  'w'
#define ANALOG_WRITE   'x'
#define TEST           'z'
#define LEFT            0
#define RIGHT           1

#endif


