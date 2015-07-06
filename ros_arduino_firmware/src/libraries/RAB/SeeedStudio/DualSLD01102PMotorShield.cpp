#include "Arduino.h"
#include "DualSLD01102PMotorShield.h"

DualSLD01102PMotorShield::DualSLD01102PMotorShield(boolean debug)
{
  _debug = debug;
  
  pinMode(_M1DIR1,OUTPUT);
  pinMode(_M1DIR2,OUTPUT);
  pinMode(_M1PWM,OUTPUT);
  pinMode(_M2DIR1,OUTPUT);
  pinMode(_M2DIR2,OUTPUT);
  pinMode(_M2PWM,OUTPUT);

  _m1_pwm = _M1MIN_PWM;
  _m1_direction = 0;
  _m2_pwm = _M2MIN_PWM;
  _m2_direction = 0;
}

void DualSLD01102PMotorShield::Stop()
{
    SetM1Velocity(0, 0);
    SetM2Velocity(0, 0);
}

void DualSLD01102PMotorShield::UpdateM1Velocity(int8_t direction, int delta)
{   
    
    //_m1_pwm = 65;
    SetM1Velocity(direction, _m1_pwm + delta);
}

void DualSLD01102PMotorShield::UpdateM2Velocity(int8_t direction, int delta)
{
        
    //_m2_pwm = 65;
    SetM2Velocity(direction, _m2_pwm + delta);
}

void DualSLD01102PMotorShield::SetM1Velocity(int8_t direction, uint8_t pwm)
{   
    _m1_direction = direction;

    if (_m1_direction > 0)
    {
        digitalWrite(_M1DIR1,LOW);
        digitalWrite(_M1DIR2,HIGH);
    }
    else if (_m1_direction < 0)
    {
        digitalWrite(_M1DIR1,HIGH);
        digitalWrite(_M1DIR2,LOW);    
    }
    else
    {
        digitalWrite(_M1DIR1,LOW);
        digitalWrite(_M1DIR2,LOW);
        pwm = 0;
    }
 
    if (pwm > _M1MAX_PWM)
    {
        _m1_pwm = _M1MAX_PWM;
    }
    else if (pwm < _M1MIN_PWM)
    {
        _m1_pwm = _M1MIN_PWM;
    }
    else
    {
    
        _m1_pwm = pwm;
    }
    
    if (_debug)
    {
        Serial.print("Dir: ");
        Serial.print(_m1_direction);
        Serial.print(", Pwm: ");
        Serial.println(_m1_pwm);
    }
    
    analogWrite(_M1PWM, _m1_pwm);  
}

void DualSLD01102PMotorShield::SetM2Velocity(int8_t direction, uint8_t pwm)
{
    _m2_direction = direction;
    
    if (_m2_direction > 0)
    {
        digitalWrite(_M2DIR1,HIGH);
        digitalWrite(_M2DIR2,LOW);    
    }
    else if (_m2_direction < 0)
    {
        digitalWrite(_M2DIR1,LOW);
        digitalWrite(_M2DIR2,HIGH);    
    }
    else
    {
        digitalWrite(_M2DIR1,LOW);
        digitalWrite(_M2DIR2,LOW);    
        pwm = 0;
    }

    if (pwm > _M2MAX_PWM)
    {
        _m2_pwm = _M2MAX_PWM;
    }
    else if (pwm < _M2MIN_PWM)
    {
        _m2_pwm = _M2MIN_PWM;
    }
    else
    {

        _m2_pwm = pwm;
    }
    
    if (_debug)
    {
        Serial.print("Dir: ");
        Serial.print(_m2_direction);
        Serial.print(", Pwm: ");
        Serial.println(_m2_pwm);
    }
    
    analogWrite(_M2PWM, _m2_pwm);  
    
}

uint8_t DualSLD01102PMotorShield::GetM1Direction()
{
    return _m1_direction;
}

uint8_t DualSLD01102PMotorShield::GetM2Direction()
{
    return _m2_direction;
}

void DualSLD01102PMotorShield::Test()
{   
  static uint8_t dir = 1;
  int i = 0;
  
  Stop();
  
  for (i = 0; i <= 255; i += 5)
  {
    Serial.print("Speed: ");
    Serial.println(i, DEC);
    SetM1Velocity(dir, i);
    SetM2Velocity(dir, i);
    delay(250);
    Stop();
    delay(500);
  }

  dir = -dir;    
}