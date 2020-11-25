#ifndef _MOTOR_h_
#define _MOTOR_h_

#include "MPin.h"

#define CLR(x, y) (x &= (~(1 << y)))
#define SET(x, y) (x |= (1 << y))
//PORTD(0->7);PORTB(8->13);PORTC(analogPins)



#define MAX_SPEED 800
#define MAX_ACCEL 50


int16_t lSpeed, rSpeed;

void InitMotors()
{
    pinMode(ENA, OUTPUT);
	pinMode(IN1, OUTPUT);
	pinMode(IN2, OUTPUT);
    
	pinMode(ENB, OUTPUT);
	pinMode(IN3, OUTPUT);
	pinMode(IN4, OUTPUT);

}

void SetMotorsSpeed(float _lSpeed, float _rSpeed)
{
    _lSpeed = constrain(_lSpeed, -MAX_SPEED, MAX_SPEED);

  if ((lSpeed - _lSpeed) > MAX_ACCEL)
    lSpeed -= MAX_ACCEL;
  else if ((lSpeed - _lSpeed) < -MAX_ACCEL)
  {
    lSpeed += MAX_ACCEL;
  }
  else
  {
    lSpeed = _lSpeed;
  }

  if (lSpeed > 0) 
  {
      
  }
}

#endif