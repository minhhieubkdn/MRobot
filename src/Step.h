//
//
//
#include <Arduino.h>
#include "MPin.h"
#include "fastio.h"

#define STEP_PER_MM 15.67f //(200*16)/(65*pi)
//#define TIMER_FREQUENCY 2000000 //2Mhz
//#define TIMER_PRESCALER 36
//#define ZERO_SPEED  65535
#define MAX_ACCEL 100
#define MAX_SPEED 800          // mm/s
#define REVERSE_MOTORS_DIRECTION // reverse both motors direction

#define RAD2GRAD 57.2957795
#define GRAD2RAD 0.01745329251994329576923690768489

#define ITERM_MAX_ERROR 30 // Iterm windup constants for PI control
#define ITERM_MAX 10000

float PID_errorSum;
float PID_errorOld = 0;
float PID_errorOld2 = 0;
float setPointOld = 0;

volatile int32_t lCurrentSteps;
volatile int32_t rCurrentSteps;
int16_t leftMotorSpeed, rightMotorSpeed; // Actual speed of motors
int8_t leftMotorDir, rightMotorDir;      // Actual direction of steppers motors

/**************************************************************************************/
/************        Initialize the Timers and Registers         ******************/
/**************************************************************************************/

// PD controller implementation(Proportional, derivative). DT in seconds
float stabilityPDControl(float DT, float input, float setPoint, float Kp, float Kd)
{
  float error;
  float output;

  error = setPoint - input;

  // Kd is implemented in two parts
  //    The biggest one using only the input (sensor) part not the SetPoint input-input(t-1).
  //    And the second using the setpoint to make it a bit more agressive   setPoint-setPoint(t-1)
  float Kd_setPoint = constrain((setPoint - setPointOld), -8, 8); // We limit the input part...
  output = Kp * error + (Kd * Kd_setPoint - Kd * (input - PID_errorOld)) / DT;
  //Serial.print(Kd*(error-PID_errorOld));Serial.print("\t");
  //PID_errorOld2 = PID_errorOld;
  PID_errorOld = input; // error for Kd is only the input component
  setPointOld = setPoint;
  return (output);
}

// PI controller implementation (Proportional, integral). DT in seconds
float speedPIControl(float DT, int16_t input, int16_t setPoint, float Kp, float Ki)
{
  int16_t error;
  float output;

  error = setPoint - input;
  PID_errorSum += constrain(error, -ITERM_MAX_ERROR, ITERM_MAX_ERROR);
  PID_errorSum = constrain(PID_errorSum, -ITERM_MAX, ITERM_MAX);

  //Serial.println(PID_errorSum);

  output = Kp * error + Ki * PID_errorSum * DT; // DT is in miliseconds...
  return (output);
}

float positionPDControl(long actualPos, long setPointPos, float Kpp, float Kdp, int16_t speedM)
{
  float output;
  float P;

  P = constrain(Kpp * float(setPointPos - actualPos), -115, 115); // Limit command
  output = P + Kdp * float(speedM);
  return (output);
}

// TIMER 1 : STEPPER MOTOR1 SPEED CONTROL
void ISR_L_MOTOR_EXE()
{
  if (leftMotorDir == 0) // If we are not moving we dont generate a pulse
    return;
  // We generate 1us STEP pulse
  WRITE(L_STEP, 1); // STEP MOTOR 1
  delayMicroseconds(1);
  if (leftMotorDir > 0)
    lCurrentSteps--;
  else
    lCurrentSteps++;
  WRITE(L_STEP, 0);
}
// TIMER 3 : STEPPER MOTOR2 SPEED CONTROL
void ISR_R_MOTOR_EXE()
{
  if (rightMotorDir == 0) // If we are not moving we dont generate a pulse
    return;
  // We generate 1us STEP pulse
  WRITE(R_STEP, 1); // STEP MOTOR 2
  delayMicroseconds(1);
  if (rightMotorDir > 0)
    rCurrentSteps--;
  else
    rCurrentSteps++;
  WRITE(R_STEP, 0);
}
void setMotorsSpeed(int16_t _lSpeed, int16_t _rSpeed)
{
  long _lPeriod;
  long _rPeriod;
  int16_t _speedInStepPerSec;

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

  _speedInStepPerSec = _lSpeed * STEP_PER_MM;
  if (_speedInStepPerSec == 0)
  {
    _lPeriod = 10;
    leftMotorDir = 0; // stop motor
  }
  else if (_speedInStepPerSec > 0)
  {
    _lPeriod = 1000000 / _speedInStepPerSec;
    leftMotorDir = 1;
#ifndef REVERSE_MOTORS_DIRECTION
    WRITE(L_DIRECTION, 1);
#else
    WRITE(L_DIRECTION, 0);
#endif
  }
  else
  {
    _lPeriod = 1000000 / -_speedInStepPerSec;
    leftMotorDir = -1;
#ifndef REVERSE_MOTORS_DIRECTION
    WRITE(L_DIRECTION, 0);
#else
    WRITE(L_DIRECTION, 1);
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

  _speedInStepPerSec = rightMotorSpeed * STEP_PER_MM;
  if (_speedInStepPerSec == 0)
  {
    _rPeriod = 10;
    rightMotorDir = 0;
  }
  else if (_speedInStepPerSec > 0)
  {
    _rPeriod = 1000000 / _speedInStepPerSec;
    rightMotorDir = 1;
#ifndef REVERSE_MOTORS_DIRECTION
    WRITE(R_DIRECTION, 1);
#else
    WRITE(R_DIRECTION, 0);
#endif
  }
  else
  {
    _rPeriod = 1000000 / -_speedInStepPerSec;
    rightMotorDir = -1;
#ifndef REVERSE_MOTORS_DIRECTION
    WRITE(R_DIRECTION, 0);
#else
    WRITE(R_DIRECTION, 1);
#endif
  }

  Timer3.setPeriod(_rPeriod);
  Timer1.setPeriod(_lPeriod);
}
void setRightMotorSpeed(int16_t _speed)
{
  long _timerPeriod;
  int16_t _speedInStepsPerSec;

  if (_speed > MAX_SPEED)
    _speed = MAX_SPEED;
  else if (_speed < -MAX_SPEED)
    _speed = -MAX_SPEED;

  if ((rightMotorSpeed - _speed) > MAX_ACCEL)
    rightMotorSpeed -= MAX_ACCEL;
  else if ((rightMotorSpeed - _speed) < -MAX_ACCEL)
  {
    rightMotorSpeed += MAX_ACCEL;
  }
  else
  {
    rightMotorSpeed = _speed;
  }

  _speedInStepsPerSec = rightMotorSpeed * STEP_PER_MM;
  if (_speedInStepsPerSec == 0)
  {
    _timerPeriod = 10;
    rightMotorDir = 0;
  }
  else if (_speedInStepsPerSec > 0)
  {
    _timerPeriod = 1000000 / _speedInStepsPerSec;
    rightMotorDir = 1;
#ifndef REVERSE_MOTORS_DIRECTION
    WRITE(R_DIRECTION, 1);
#else
    WRITE(R_DIRECTION, 0);
#endif
  }
  else
  {
    _timerPeriod = 1000000 / -_speedInStepsPerSec;
    rightMotorDir = -1;
#ifndef REVERSE_MOTORS_DIRECTION
    WRITE(R_DIRECTION, 0);
#else
    WRITE(R_DIRECTION, 1);
#endif
  }

  //Timer3.pause();
  Timer3.setPeriod(_timerPeriod);
  //Timer3.refresh();
  //Timer3.resume();
}

void setLeftMotorSpeed(int16_t _speed)
{
  long _timerOverFlowValue;
  int16_t _speedInStepPerSec;

  if ((leftMotorSpeed - _speed) > MAX_ACCEL)
  {
    leftMotorSpeed -= MAX_ACCEL;
  }
  else if ((leftMotorSpeed - _speed) < -MAX_ACCEL)
  {
    leftMotorSpeed += MAX_ACCEL;
  }
  else
  {
    leftMotorSpeed = _speed;
  }

  _speedInStepPerSec = _speed * STEP_PER_MM;
  if (_speedInStepPerSec == 0)
  {
    _timerOverFlowValue = 10;
    leftMotorDir = 0; // stop motor
  }
  else if (_speedInStepPerSec > 0)
  {
    _timerOverFlowValue = 1000000 / _speedInStepPerSec;
    leftMotorDir = 1;
#ifndef REVERSE_MOTORS_DIRECTION
    WRITE(L_DIRECTION, 1);
#else
    WRITE(L_DIRECTION, 0);
#endif
  }
  else
  {
    _timerOverFlowValue = 1000000 / -_speedInStepPerSec;
    leftMotorDir = -1;
#ifndef REVERSE_MOTORS_DIRECTION
    WRITE(L_DIRECTION, 0);
#else
    WRITE(L_DIRECTION, 1);
#endif
  }
  //Timer1.pause();
  Timer1.setPeriod(_timerOverFlowValue);
  //Timer1.refresh();
  //Timer1.resume();

  //Serial.println(_timerOverFlowValue);
}

void InitMotors()
{
  /****************            mark all driver pins as Output             ******************/
  pinMode(L_DIRECTION, OUTPUT);
  pinMode(L_STEP, OUTPUT);
  pinMode(L_ENABLE, OUTPUT);
  pinMode(R_DIRECTION, OUTPUT);
  pinMode(R_STEP, OUTPUT);
  pinMode(R_ENABLE, OUTPUT);

  /********  Specific Timers & Registers for the atmega328P (Promini)   ************/
  digitalWrite(L_ENABLE, HIGH); // Disable motors
  digitalWrite(R_ENABLE, HIGH);

  Timer1.pause();
  Timer1.setPeriod(10);
  Timer1.setMode(TIMER_CH1, TIMER_OUTPUT_COMPARE);
  Timer1.setCompare(TIMER_CH1, 1);
  Timer1.attachInterrupt(TIMER_CH1, ISR_L_MOTOR_EXE);
  Timer1.refresh();

  Timer3.pause();
  Timer3.setPeriod(10);
  Timer3.setMode(TIMER_CH1, TIMER_OUTPUT_COMPARE);
  Timer3.setCompare(TIMER_CH1, 1);
  Timer3.attachInterrupt(TIMER_CH1, ISR_R_MOTOR_EXE);
  Timer3.refresh();

  digitalWrite(L_ENABLE, LOW); // Enable stepper drivers
  digitalWrite(R_ENABLE, LOW);

  Timer1.resume();
  Timer3.resume();
}