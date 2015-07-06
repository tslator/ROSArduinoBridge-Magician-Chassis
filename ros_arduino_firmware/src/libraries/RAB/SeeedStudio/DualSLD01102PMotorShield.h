#ifndef DualSLD01102PMotorShield_h
#define DualSLD01102PMotorShield_h

#include "Arduino.h"

class DualSLD01102PMotorShield
{
    public:
        DualSLD01102PMotorShield(boolean debug=false);
        void Stop();
        void UpdateM1Velocity(int8_t direction, int delta);
        void UpdateM2Velocity(int8_t direction, int delta);
        void SetM1Velocity(int8_t direction, uint8_t speed);
        void SetM2Velocity(int8_t direction, uint8_t speed);
        
        uint8_t GetM1Direction();
        uint8_t GetM2Direction();
        int8_t volatile _m1_direction;
        int8_t volatile _m2_direction;
        uint8_t _m1_pwm;
        uint8_t _m2_pwm;
        
        void Test();
        
    private:
        static const uint8_t _M1MIN_PWM = 0;
        static const uint8_t _M1MAX_PWM = 255;
        static const uint8_t _M2MIN_PWM = 0;
        static const uint8_t _M2MAX_PWM = 255;
        
        static const uint8_t _M1DIR1 = 8;
        static const uint8_t _M1DIR2 = 11;
        static const uint8_t _M1PWM = 9;
        static const uint8_t _M2DIR1 = 12;
        static const uint8_t _M2DIR2 = 13;
        static const uint8_t _M2PWM = 10;
        
        
        
        boolean _debug;
};
#endif
