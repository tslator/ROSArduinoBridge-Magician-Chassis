#include "Arduino.h"
#include "DualSEN12617Encoder.h"

#define LEFT_MIN_HIGH_TIME 5
#define RIGHT_MIN_HIGH_TIME 5

static volatile int32_t left_count = 0;
static uint8_t left_debounce = 15; // in milliseconds
static volatile int32_t right_count = 0;
static uint8_t right_debounce = 15; // in milliseconds
static DualSLD01102PMotorShield *ref_motor;

static void LeftEncoderCount()
// If pin transitions from low to high then
//    If captured current millis + minimum delay < millis then
//        Tick the count
//        Note: Increment the count by adding the direction (1, -1, 0)
//    End if
// Else If transitions from high to low then
//    capture current millis
// End if
{
    static uint8_t last_pin_state = 0;
    static uint32_t last_rise_time = 0;
    uint8_t new_pin_state = PIND & B00000100;
    
    if (!last_pin_state && new_pin_state)
    {
        last_rise_time = millis();
    }
    else if (last_pin_state && !new_pin_state);
    {
        if (last_rise_time + LEFT_MIN_HIGH_TIME < millis())
        {
            // Note: There were problems calling GetM1Direction from this interrupt
            // that caused the count to go wacky, i.e., worked fine going forward but
            // gave ridiculous values in reverse.  Moving the member variable to be
            // public seems to have fixed it.
            // Also add volatile to its definition.  Possibly inline would help as well.
            //left_count += ref_motor->GetM1Direction();
            left_count += ref_motor->_m1_direction;
        }
    }
    last_pin_state = new_pin_state;
}

static void RightEncoderCount()
// If pin transitions from low to high then
//    If captured current millis + minimum delay < millis then
//        Tick the count
//        Note: Increment the count by adding the direction (1, -1, 0)
//    End if
// Else If transitions from high to low then
//    capture current millis
// End if
{
    static uint8_t last_pin_state = 0;
    static uint32_t last_rise_time = 0;    
    uint8_t new_pin_state = PIND & B00001000;
    
    if (!last_pin_state && new_pin_state)
    {
        last_rise_time = millis();
    }
    else if (last_pin_state && !new_pin_state);
    {
        if (last_rise_time + RIGHT_MIN_HIGH_TIME < millis())
        {
            // Note: There were problems calling GetM1Direction from this interrupt
            // that caused the count to go wacky, i.e., worked fine going forward but
            // gave ridiculous values in reverse.  Moving the member variable to be
            // public seems to have fixed it.
            // Also add volatile to its definition.  Possibly inline would help as well.
            //right_count += ref_motor->GetM2Direction();
            right_count += ref_motor->_m2_direction;
        }
    }
    last_pin_state = new_pin_state;
}


DualSEN12617Encoder::DualSEN12617Encoder(DualSLD01102PMotorShield *motor, boolean debug)
{
    _debug = debug;
    
    // In order to access the motor direction it is necessary to make the motor available in module scope
    ref_motor = motor;
    pinMode(2, INPUT);
    pinMode(3, INPUT);
    digitalWrite(2, HIGH);
    digitalWrite(3, HIGH);
    attachInterrupt(0, LeftEncoderCount, CHANGE); // Interrupt 0 is on pin 2
    attachInterrupt(1, RightEncoderCount, CHANGE); // Interrupt 1 is on pin 3
}

void DualSEN12617Encoder::GetCountDelta(uint32_t *left, uint32_t *right)
{
    static int32_t last_left_count = 0;
    static int32_t last_right_count = 0;
    static int32_t left_delta = 0;
    static int32_t right_delta = 0;
    static uint32_t last_millis = 0;

    left_delta = abs(abs(left_count) - abs(last_left_count));
    right_delta = abs(abs(right_count) - abs(last_right_count));
    
    if (_debug)
    {
        Serial.print("lc: ");
        Serial.print(left_count);
        Serial.print(",llc: ");
        Serial.print(last_left_count);
        Serial.print(",ld: ");
        Serial.print(left_delta);
        Serial.print("rc: ");
        Serial.print(right_count);
        Serial.print(",lrc: ");
        Serial.print(last_right_count);
        Serial.print(", rd: ");
        Serial.println(right_delta);
    }
    
    last_left_count = left_count;
    last_right_count = right_count;
    
    *left = left_delta;
    *right = right_delta;

}

void DualSEN12617Encoder::GetCounts(int32_t *left, int32_t *right)
{
    *left = left_count;
    *right = right_count;
}

void DualSEN12617Encoder::Reset()
{
    left_count = 0;
    right_count = 0;
}

void DualSEN12617Encoder::Test()
{
    static uint32_t last = millis();
    static uint32_t last_left = 0;
    static uint32_t last_right = 0;
    uint32_t now = millis();
    int32_t left_ticks_per_cycle;
    int32_t right_ticks_per_cycle;
    /*
    Serial.print("Left: ");
    Serial.print(left_count);
    Serial.print(", Right: ");
    Serial.println(right_count);
    */
    
    if ( (now - last) > 100)
    {
        /*
        Serial.print("Left: ");
        left_ticks_per_cycle = abs(left_count - last_left)*10;
        Serial.print(left_ticks_per_cycle);
        Serial.print(", Right: ");
        right_ticks_per_cycle = abs(right_count - last_right)*10;
        Serial.println(right_ticks_per_cycle);
        */
        last_left = left_count;
        last_right = right_count;
        last = now;
    }
}
