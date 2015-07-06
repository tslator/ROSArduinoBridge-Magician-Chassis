import time
import signal
import sys
from pymata import PyMata
import random


def Constrain(value, lower_limit, upper_limit):
    if value > upper_limit:
        value = upper_limit
    elif value < lower_limit:
        value = lower_limit
    return value

class RosArduinoBridgePyMataController:
    def __init__(self, channel, params):
        self.channel = channel
        self.channel.rab_config( params['pid rate'] )
        self.channel.rab_request_library_version()
        print "RAB Library Version:", self.channel.get_rab_version()
        
        # TODO:
        #  Add support for Pan-Tilt using servos
            
    def __drive(self, speeds):
        # Note: Speeds are specified as ticks per pid cycle.  The value is floating point.
        # Pass the value over the channel as fixed point.
        
        speed_list = []
        
        for i in range(len(speeds)):
            # Convert to fixed point, one significant digit
            fixed_pt_value = int(speeds[i] * 10)
            # Scale for transmission
            scaled_value = fixed_pt_value + 128
            speed_list.append(scaled_value)
        
        self.channel.rab_drive( speed_list )
        
    def DriveMotors(self, speeds):
        self.__drive( [speeds['left'], speeds['right']] )

    def StopMotors(self):        
        self.__drive( [0, 0] )
        
    def ReadEncoders(self):
        # Note: This is a blocking call.  It would be nice to have the counts transmitted on a 
        # regular basis (when they change) and then just return the latest.
        self.channel.rab_read_encoder_count();
        return self.channel.get_rab_encoder_count()
        
    def ResetEncoders(self):
        self.channel.rab_reset_encoder()
            
    def UpdatePid(self):
        #self.channel.rab_update_pid(...)
        pass
        
    def Close(self):
        pass
        
    def ServoWrite(self, id, position):
        pass
        
    def ServoRead(self, id):
        pass

channel = None
rab = None

if __name__ == "__main__":
    
    def signal_handler(sig, frame):
        global channel
        global rab
        print('You pressed Ctrl+C!!!!')
        if channel is not None:
            rab.StopMotors()
            time.sleep(1)
            channel.reset()
        sys.exit(0)

    def callback(data):
        print data

    def TestMotorsEncoders():
        rab.ResetEncoders()
        print "Encoders: ", rab.ReadEncoders()
        rab.DriveMotors( {'left' : 6.0 , 'right' : 6.0 } )
        time.sleep(5)
        rab.DriveMotors( {'left' : 0.0 , 'right' : 0.0 } )
        time.sleep(2)
        print "Encoders: ", rab.ReadEncoders()
        rab.DriveMotors( {'left' : -6.0 , 'right' : -6.0 } )
        time.sleep(5)
        rab.DriveMotors( {'left' : 0.0 , 'right' : 0.0 } )
        time.sleep(2)
        print "Encoders: ", rab.ReadEncoders()
        rab.DriveMotors( {'left' : -6.0 , 'right' : 6.0 } )
        time.sleep(5)
        rab.DriveMotors( {'left' : 0.0 , 'right' : 0.0 } )
        time.sleep(2)
        print "Encoders: ", rab.ReadEncoders()
        rab.DriveMotors( {'left' : 6.0 , 'right' : -6.0 } )
        time.sleep(5)
        rab.DriveMotors( {'left' : 0.0 , 'right' : 0.0 } )
        time.sleep(2)
        print "Encoders: ", rab.ReadEncoders()
        
        rab.StopMotors()
        
    def TestSpeedChange():
        for speed in range(0, 100, 1):
            print "Speed: ", float(speed)/10
            rab.DriveMotors( {'left' : float(speed)/10 , 'right' : float(speed)/10 } )
            time.sleep(1)
            
        rab.StopMotors()
        
        for speed in range(100, 0, -1):
            print "Speed: ", float(speed)/10
            rab.DriveMotors( {'left' : float(speed)/10 , 'right' : float(speed)/10 } )
            time.sleep(1)
            
        rab.StopMotors()

    config = {
        'pid rate' : 10
    }
    
    signal.signal(signal.SIGINT, signal_handler)
    
    # Create PyMata instance
    channel = PyMata("COM10")
    rab = RosArduinoBridgePyMataController(channel, config)
    
    # The BaseController converts Twist linear and angular commands into 'ticks per PID internal'
    # The ROS configuration knows, via parameters, the dimensions of the wheel, wheel base, and counts per revolution
    #    - The PID interval is 10hz or 100 ms
    #
    # 1 RPS:
    #    - 1 rev per second = 16 ticks per second
    #    - 1.6 ticks per PID interval
    #
    # 5 RPS:
    #    - 5 rev per second = 80 ticks per second
    #    - 8.0 ticks per PID interval
    #
    # 10 RPS:
    #    - 10 rev per second = 160 ticks per second
    #    - 16 ticks per PID interval
    
    #TestMotorsEncoders()
    TestSpeedChange()
