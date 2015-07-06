#include "Arduino.h"
#include "rab.h"

#include "DualSLD01102PMotorShield.h"
#include "DualSEN12617Encoder.h"
#include "PID_v1.h"

RAB::RAB(uint8_t pid_rate, boolean debug)
{
    _debug = debug;
    
    // Instantiate the Motor Shield
    _motor_shield = new DualSLD01102PMotorShield(_debug);
    _motor_shield->Stop();
    
    // Instantiate the Encoder (the motor is needed to know direction)
    _encoder = new DualSEN12617Encoder(_motor_shield, _debug);
    _encoder->Reset();
    
    // Instantiate the PIDs
    _lkp = 3.0;
    _lki = 1.0;
    _lkd = 0.01;
    _rkp = 3.0;
    _rki = 1.0;
    _rkd = 0.01;
    
    _left_pid = new PID(&_left_input, &_left_output, &_left_set_point, _lkp, _lki, _lkd, DIRECT, _debug);
    _left_pid->SetOutputLimits(-255, 255);
    _left_pid->SetSampleTime(_pid_sample_time_ms);
    
    _right_pid = new PID(&_right_input, &_right_output, &_right_set_point, _rkp, _rki, _rkd, DIRECT, _debug);
    _right_pid->SetOutputLimits(-255, 255);
    _right_pid->SetSampleTime(_pid_sample_time_ms);
    
    _left_pid->SetMode(AUTOMATIC);
    _right_pid->SetMode(AUTOMATIC);
    
    _left_pid->Reset();
    _right_pid->Reset();
    
    _left_set_point = 0;
    _right_set_point = 0;

    // Set pid rate and sample time
    _pid_rate = pid_rate;
    _pid_sample_time_ms = 1000/pid_rate;

    // Set initial direction
    _left_direction = 0;
    _right_direction = 0;
    
}

int8_t RAB::SetDirection(int8_t direction)
{
    if (direction > 0)
    {
        return 1;
    }
    else if (direction < 0)
    {
        return -1;
    }
    else
    {
        return 0;
    }
}

void RAB::Drive(double left, double right)
{   
    /* Experimentally, this configuration support -10 .. 10 ticks per pid cycle */
    left = constrain(left, _MIN_TICKS_PER_PID, _MAX_TICKS_PER_PID);
    right = constrain(right, _MIN_TICKS_PER_PID, _MAX_TICKS_PER_PID);
    
    /* Convert ticks per PID cycle into ticks per second */
    _left_set_point = abs(left); 
    _right_set_point = abs(right);
    
    if (_left_set_point == 0)
    {
        _left_pid->Reset();
    }
    if (_right_set_point == 0)
    {
        _right_pid->Reset();
    }
    
    _left_direction = SetDirection(left);
    _right_direction = SetDirection(right);
    
    if (_debug)
    {
        Serial.print("lsp: ");
        Serial.print(_left_set_point);
        Serial.print(", ld: ");
        Serial.print(_left_direction);
        Serial.print(", rsp: ");
        Serial.print(_right_set_point);
        Serial.print(", rd: ");
        Serial.println(_right_direction);
    }
}

void RAB::FilterCounts(uint32_t *left_delta, uint32_t *right_delta, double *left, double *right)
{
    #define BUFFER_SIZE 1
    static uint32_t left_buf[BUFFER_SIZE] = {0};
    static uint32_t right_buf[BUFFER_SIZE] = {0};
    static uint8_t count = 0;
    uint32_t left_sum = 0;
    uint32_t right_sum = 0;
    uint8_t ii;
    
    left_buf[count] = *left_delta;
    right_buf[count] = *right_delta;
    count = (count + 1) % BUFFER_SIZE;
    
    left_sum = 0;
    right_sum = 0;
    
    for (ii = 0; ii < BUFFER_SIZE; ++ii)
    {
        left_sum += left_buf[ii];
        right_sum += right_buf[ii];
    }
    
    *left = ((double)left_sum/BUFFER_SIZE);
    *right = ((double)right_sum/BUFFER_SIZE);
}

void RAB::GetInput()
{
    uint32_t left_delta;
    uint32_t right_delta;
    double left;
    double right;
    
    /* Count delta is in ticks per second */
    _encoder->GetCountDelta(&left_delta, &right_delta);
    FilterCounts(&left_delta, &right_delta, &left, &right);
    /*
    Serial.print("left: ");
    Serial.print(left);
    Serial.print(", right: ");
    Serial.println(right);
    */
    
    _left_input = left;
    _right_input = right;
}

void RAB::Calculate()
{
    
    _left_pid->Compute();
    _right_pid->Compute();
    /*
    Serial.print("LSetPoint: ");
    Serial.print(_left_set_point);
    Serial.print(", LInput: ");
    Serial.print(_left_input);
    Serial.print(", LOutput ");
    Serial.println(_left_output);
    Serial.print("RSetPoint: ");
    Serial.print(_right_set_point);
    Serial.print(", RInput: ");
    Serial.print(_right_input);
    Serial.print(", ROutput ");
    Serial.println(_right_output);
    */
}

void RAB::SetOutput()
{
    
    _motor_shield->UpdateM1Velocity(_left_direction, _left_output);
    _motor_shield->UpdateM2Velocity(_right_direction, _right_output);
    /*
    Serial.print("m1: ");
    Serial.print(_motor_shield->_m1_pwm);
    Serial.print(", m2: ");
    Serial.println(_motor_shield->_m2_pwm);
    */
}

void RAB::Update()
{   
    static uint32_t last = millis();
    uint32_t now = millis();
    if ( (now - last) > _pid_sample_time_ms)
    {
        GetInput();
        Calculate();
        SetOutput();
        
        GetEncoderCount();
        
        last = now;
    }
}

void RAB::GetEncoderCount()
{
    _encoder->GetCounts(&left_encoder_count, &right_encoder_count);
}

void RAB::ResetEncoderCount()
{
    _encoder->Reset();
    left_encoder_count = 0;
    right_encoder_count = 0;
}

void RAB::UpdatePid(uint8_t lkp, uint8_t lki, uint8_t lkd, uint8_t rkp, uint8_t rki, uint8_t rkd)
{
    _left_pid->SetTunings( (double (lkp))/10, (double (lki))/10, (double (lkd))/10 );
    _right_pid->SetTunings( (double (rkp))/10, (double (rki))/10, (double (rkd))/10 );    
}

void RAB::Stop()
{
    _motor_shield->Stop();
}

void RAB::Show()
{    
    // A way to print debug or return detailed status information via firmata.
}

void RAB::SetPidRate(uint8_t rate)
{
    _left_pid->SetSampleTime(1000/rate);
    _right_pid->SetSampleTime(1000/rate);
}

//------------------------------------------------------------------------------
// Test Methods
//------------------------------------------------------------------------------

void RAB::MotorTest()
{
    _motor_shield->Test();
}

void RAB::EncoderTest(int32_t left, int32_t right)
// Drive each motor at a specific pwm
// Display the encoder count delta for each motor
{
    static uint8_t left_pwm = 0;
    static uint8_t right_pwm = 0;
    static uint8_t first_time = 1;
    static uint32_t cycle_count = 0;
    static uint8_t dir = 1;
    uint32_t left_delta;
    uint32_t right_delta;
    static uint32_t last = millis();
    
    if (first_time)
    {
        _encoder->Reset();
        first_time = 0;
    }
    
    uint32_t now = millis();
    if ( (now - last) > _pid_sample_time_ms)
    {
        if (left != left_pwm)
        {
            left_pwm = left;
        }
        if (right != right_pwm)
        {
            right_pwm = right;
        }
        
        _motor_shield->SetM1Velocity(dir, left_pwm);
        _motor_shield->SetM2Velocity(dir, right_pwm);
            
        _encoder->GetCountDelta(&left_delta, &right_delta);
        if (_debug)
        {
            Serial.print("Dir: ");
            Serial.print(dir);
            Serial.print(", Pwm: (");
            Serial.print(left_pwm);
            Serial.print(",");
            Serial.print(right_pwm);
            Serial.print(")");
            Serial.print(", Left: ");
            Serial.print(left_delta);
            Serial.print(", Right: ");
            Serial.println(right_delta);
        }
        
        last = now;
    }
    
}

void RAB::PidTest()
{
    Drive( 5, 5 );
    /*
    Serial.print("SetPoint: (");
    Serial.print(_left_set_point);
    Serial.print(",");
    Serial.print(_right_set_point);
    Serial.println(")");
    */
    GetInput();
    /*
    Serial.print("Input: (");
    Serial.print(_left_input);
    Serial.print(",");
    Serial.print(_right_input);
    Serial.println(")");
    */
    Calculate();    
    /*
    Serial.print("Output: (");
    Serial.print(_left_output);
    Serial.print(",");
    Serial.print(_right_output);
    Serial.println(")");
    */
    SetOutput();
}

int RAB::Version()
{
    return 1;
}