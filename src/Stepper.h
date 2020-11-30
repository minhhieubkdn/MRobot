//
//
//
#include <Arduino.h>
#include "MPin.h"

#define STEP_PER_MM 15.67f //(200*16)/(65*pi)
#define MAX_ACCEL 50
#define MAX_SPEED 800          // mm/s
//#define REVERSE_MOTORS_DIRECTION // reverse both motors direction

#define SPEED_TO_PERIOD(x) (1000000.0 / (STEP_PER_MM * x))

#define COMPARE_VALUE_TIMER OCR1A
#define TurnOnTimer1 (TIMSK1 |= (1 << OCIE1A))
#define TurnOffTimer1 (TIMSK1 &= ~(1 << OCIE1A))

#define CLR(x, y) (x &= (~(1 << y)))
#define SET(x, y) (x |= (1 << y))
//PORTD(0->7);PORTB(8->13);PORTC(analogPins)
#define L_STEP_ON SET(PORTD, 5)  //D5
#define L_STEP_OFF CLR(PORTD, 5)
#define L_DIR_ON SET(PORTD, 7)  //D7
#define L_DIR_OFF CLR(PORTD, 7)
#define R_STEP_ON SET(PORTD, 6)  //D6
#define R_STEP_OFF CLR(PORTD, 6)
#define R_DIR_ON SET(PORTB, 1)  //D8
#define R_DIR_OFF CLR(PORTB, 1)

int16_t leftMotorSpeed, rightMotorSpeed; // Actual speed of motors
int leftMotorDir, rightMotorDir;      // Actual direction of steppers motors

int16_t lCounter, rCounter;
int16_t lDesiredCounter, rDesiredCounter;

void ISR_L_MOTOR_EXE()
{
  if (leftMotorDir == 0) // If we are not moving we dont generate a pulse
    return;
  // We generate 1us STEP pulse
  L_STEP_ON; // STEP MOTOR 1
  delayMicroseconds(1);
  L_STEP_OFF;
}

void ISR_R_MOTOR_EXE()
{
  if (rightMotorDir == 0) // If we are not moving we dont generate a pulse
    return;
  // We generate 1us STEP pulse
  R_STEP_ON; // STEP MOTOR 2
  delayMicroseconds(1);
  R_STEP_OFF;
}

void setMotorsSpeed(int16_t _lSpeed, int16_t _rSpeed)
{
  if (_lSpeed > MAX_SPEED)
    _lSpeed = MAX_SPEED;
  else if (_lSpeed < -MAX_SPEED)
    _lSpeed = -MAX_SPEED;

  if ((leftMotorSpeed - _lSpeed) > MAX_ACCEL)
  {
    leftMotorSpeed -= MAX_ACCEL;
  }
  else if ((leftMotorSpeed - _lSpeed) < -MAX_ACCEL)
  {
    leftMotorSpeed += MAX_ACCEL;
  }
  else
  {
    leftMotorSpeed = _lSpeed;
  }

  if (leftMotorSpeed == 0)
  {
    leftMotorDir = 0; // stop motor
  }
  else if (leftMotorSpeed > 0)
  {
    lDesiredCounter = SPEED_TO_PERIOD(leftMotorSpeed)/10;
    leftMotorDir = 1;
#ifndef REVERSE_MOTORS_DIRECTION
    PORTD &= 0b01111111;
#else
    PORTD |= 0b10000000;
#endif
  }
  else
  {
    lDesiredCounter = -SPEED_TO_PERIOD(leftMotorSpeed)/10;
    leftMotorDir = -1;
#ifndef REVERSE_MOTORS_DIRECTION
    PORTD |= 0b10000000;
#else
    PORTD &= 0b01111111;
#endif
  }

  if (_rSpeed > MAX_SPEED)
    _rSpeed = MAX_SPEED;
  else if (_rSpeed < -MAX_SPEED)
    _rSpeed = -MAX_SPEED;

  if ((rightMotorSpeed - _rSpeed) > MAX_ACCEL)
    rightMotorSpeed -= MAX_ACCEL;
  else if ((rightMotorSpeed - _rSpeed) < -MAX_ACCEL)
  {
    rightMotorSpeed += MAX_ACCEL;
  }
  else
  {
    rightMotorSpeed = _rSpeed;
  }

  if (rightMotorSpeed == 0)
  {
    rightMotorDir = 0;
  }
  else if (rightMotorSpeed > 0)
  {
    rDesiredCounter = SPEED_TO_PERIOD(rightMotorSpeed)/10;
    rightMotorDir = 1;
#ifndef REVERSE_MOTORS_DIRECTION
    PORTB |= 0b00000001;
#else
    PORTB &= 0b11111110;
#endif
  }
  else
  {
    rDesiredCounter = -SPEED_TO_PERIOD(rightMotorSpeed)/10;
    rightMotorDir = -1;
#ifndef REVERSE_MOTORS_DIRECTION
    PORTB &= 0b11111110;
#else
    PORTB |= 0b00000001;
#endif
  }

  
  
}

void TimerInit()
{
	noInterrupts();

	// Reset register relate to Timer 1
	// Reset register relate
	TCCR1A = TCCR1B = TCNT1 = 0;
	// Set CTC mode to Timer 1
	TCCR1B |= (1 << WGM12);
	// Set prescaler 1 to Timer 1
	TCCR1B |= (1 << CS10);
	//Normal port operation, OCxA disconnected
	TCCR1A &= ~((1 << COM1A1) | (1 << COM1A0) | (1 << COM1B1) | (1 << COM1B0));

	interrupts();
}
void setMicrosCycle(int intCycle)  //micro second
{
	int prescaler;

	if (intCycle > 4000)
	{
		TCCR1B |= (1 << CS11);
		TCCR1B &= ~(1 << CS10);
		prescaler = 8;
	}
	else
	{
		TCCR1B &= ~(1 << CS11);
		TCCR1B |= (1 << CS10);
		prescaler = 1;
	}

	COMPARE_VALUE_TIMER = roundf(intCycle * 16 / prescaler) - 1;
	//COMPARE_VALUE_TIMER = roundf(intCycle * F_CPU / (1000000.0 * prescaler)) - 1;
}

void InitMotors()
{
  pinMode(L_DIRECTION, OUTPUT);
  pinMode(L_STEP, OUTPUT);
  pinMode(L_ENABLE, OUTPUT);
  pinMode(R_DIRECTION, OUTPUT);
  pinMode(R_STEP, OUTPUT);
  pinMode(R_ENABLE, OUTPUT);

  digitalWrite(L_ENABLE, LOW); // Enable motors

  TimerInit();
  setMicrosCycle(20);
  TurnOnTimer1;
}

ISR(TIMER1_COMPA_vect)
{
  lCounter++;
  if (lCounter > lDesiredCounter)
  {
    lCounter = 0;
  }
  else if (lCounter == 1) 
    ISR_L_MOTOR_EXE();

  rCounter++;
  if (rCounter > rDesiredCounter)
  {
    rCounter = 0;
  }
  else if (rCounter == 1)
    ISR_R_MOTOR_EXE();
}
