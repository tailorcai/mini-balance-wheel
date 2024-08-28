/******************************************************************************

******************************************************************************/

#include "AT8236_ESP32.h"
#include <Arduino.h>
#include <driver/ledc.h>
Motor_8236::Motor_8236(int In1pin, int In2pin, int offset, int freq, int resolution, int channel_pin)
{
  In1 = In1pin;
  In2 = In2pin;
//   PWM = PWMpin;
//   Standby = STBYpin;
  Offset = offset;
  Channel=channel_pin*2;

  pinMode(In1, OUTPUT);
  pinMode(In2, OUTPUT);
//   pinMode(PWMpin, OUTPUT);
  pinMode(Standby, OUTPUT);

  ledcSetup(Channel, freq, resolution);
  ledcSetup(Channel+1, freq, resolution);
  ledcAttachPin(In1, Channel);
  ledcAttachPin(In2, Channel+1);

  ledcWrite(Channel,0);
  ledcWrite(Channel+1,0);

  // analogWrite(In1, 0);
  // analogWrite(In2, 0);
}

void Motor_8236::drive(int speed)
{
//   digitalWrite(Standby, HIGH);
  speed = speed * Offset;
  if (speed>=0) fwd(speed);
  else rev(-speed);
}
void Motor_8236::drive(int speed, int duration)
{
  drive(speed);
  delay(duration);
}

void Motor_8236::fwd(int speed)
{
   // digitalWrite(In1, HIGH);
   // digitalWrite(In2, LOW);
   ledcWrite(Channel, speed);
   ledcWrite(Channel+1, 0);

    // analogWrite(In1, 255);
    // analogWrite(In2, 255-speed);
}

void Motor_8236::rev(int speed)
{
   // digitalWrite(In1, LOW);
   // digitalWrite(In2, HIGH);
   ledcWrite(Channel, 0);
   ledcWrite(Channel+1, speed);
  //  analogWrite(In1, 255-speed);
  //  analogWrite(In2, 255);
}

void Motor_8236::brake()
{
  //  analogWrite(In1, 255);
  //  analogWrite(In2, 255);
   ledcWrite(Channel, 0);
   ledcWrite(Channel+1, 0);
}

void Motor_8236::standby()
{
   // digitalWrite(Standby, LOW);
}
