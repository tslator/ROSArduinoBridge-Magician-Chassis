#ifndef RAB_h
#define RAB_h

#include "Arduino.h"
#include <DualSLD01102PMotorShield.h>
#include <DualSEN12617Encoder.h>
#include <PID_v1.h>


class RAB
{
    // This class is composed of three objects;
    //   - Seeed Motor Shield v2
    //   - Sparkfun SEN12617 dual encoder
    //   - Arduino PID
    // This class accepts as a parameter the PID rate which is used to drive the PID
    // This class must return the encoder counts

    public:
    
        RAB(uint8_t pid_rate=10, boolean debug=false);
        void Drive(double left, double right);
        void Stop();
        void Update();
        void ResetEncoderCount();
        void UpdatePid(uint8_t lkp, uint8_t lki, uint8_t lkd, uint8_t rkp, uint8_t rki, uint8_t rkd);
        void SetPidRate(uint8_t rate);
        void Show();
        int Version();
        
        int32_t left_encoder_count;
        int32_t right_encoder_count;
        
        // Test Methods
        void MotorTest();
        void EncoderTest(int32_t left, int32_t right);
        void PidTest();
        
    private:
        static const uint16_t _MS_PER_SECOND = 1000;
        static const double _MIN_TICKS_PER_PID = -10.0;
        static const double _MAX_TICKS_PER_PID = +10.0;
    
        void GetInput();
        void Calculate();
        void SetOutput();
        void GetEncoderCount();
        int8_t SetDirection(int8_t direction);
        void FilterCounts(uint32_t *left_delta, uint32_t *right_delta, double *left, double *right);        

        boolean _debug;
        
        DualSLD01102PMotorShield *_motor_shield;
        DualSEN12617Encoder      *_encoder;
        double _left_set_point, _left_input, _left_output;
        double _right_set_point, _right_input, _right_output;
        double _lkp, _lki, _lkd;
        double _rkp, _rki, _rkd;
        PID *_left_pid;
        PID *_right_pid;
        
        int8_t _left_direction;
        int8_t _right_direction;
        uint8_t _pid_rate;
        uint32_t _pid_sample_time_ms;
};

#endif
