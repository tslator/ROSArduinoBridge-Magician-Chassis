/* This test program exercises the DualSLD01102PMotorShield class by
   running the motors forwards and reverse at different speeds
 */

#include <DualSLD01102PMotorShield.h>

DualSLD01102PMotorShield ms = DualSLD01102PMotorShield(false);
int dir = 1;

void setup()
{
  Serial.begin(57600);
}

void loop()
{
  int i = 0;
  
  ms.Stop();
  
  for (i = 0; i <= 255; i += 5)
  {
    Serial.print("Speed: ");
    Serial.println(i, DEC);
    ms.SetM1Velocity(dir, i);
    ms.SetM2Velocity(dir, i);
    delay(250);
    ms.Stop();
    delay(500);
  }

  dir = -dir;
}