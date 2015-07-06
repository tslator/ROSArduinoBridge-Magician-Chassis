#ifndef DualSEN12617Encoder_h
#define DualSEN12617Encoder_h

#include "Arduino.h"
#include "DualSLD01102PMotorShield.h"

class DualSEN12617Encoder
{
    public:
        DualSEN12617Encoder(DualSLD01102PMotorShield *motor, boolean debug=false);
        void GetCountDelta(uint32_t *left, uint32_t *right);
        void GetCounts(int32_t *left, int32_t *right);
        void Reset();
        void Test();
    private:    
        static const uint16_t _MS_PER_SECOND = 1000;
        
        static const uint8_t _INTR0 = 0;
        static const uint8_t _INTR1 = 1;
        
        boolean _debug;
        
};
#endif