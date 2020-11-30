#include <Arduino.h>
#include "MPin.h"
#include "MEEPROM.h"
#include "MMPU6050.h"
#include "Step.h"

#define DEBUG 3

#define KP 5
#define KI 0
#define KD 0

#define KPD 0.1
#define KID 0
#define KDD 0

#define PULSE2DISTANCE 0.01715 // cm/us
#define NOISE 0.5				   //+- mm/s
#define DEBUG_PORT Serial
#define COMMAND_PORT Serial1

#define BALANCE_ANGLE 0

bool isStringCompleted = false;
String inputString;

uint32_t oldMicros;
uint32_t currentMicros;
float dt;

float sLastError, sSecondToLastError;
float Kp = KP, Ki = KI, Kd = KD;
float sPIDout = 0, sLastPIDout = 0;
float speedOut, currentAngle;

float dLastError, dSecondToLastError;
float Kpd = KPD, Kid = KID, Kdd = KDD;
float dPIDout = 0, dlastPIDout = 0;
float dirOut = 0, currentDir;

float targetAngle = BALANCE_ANGLE;
float targetDir = 0;

float lMotorSpeed, rMotorSpeed;
float steering = 0;

bool BATlow = false;
float y, p, r;

bool isWaitingFor10Secs = true;
uint32_t beginMicros;

uint32_t checkObstacleTimer;
uint8_t obstacleNum = 0;
long duration, distance;

float I_mem,error_old;

float CalcuPID(float dt_, float error_, float set_point_) {
	float pid_out_, error;

	error = error_ - set_point_;
	I_mem += error * Ki;
	if (I_mem > MAX_SPEED) I_mem = MAX_SPEED;
	else if (I_mem < -MAX_SPEED) I_mem = -MAX_SPEED;
	pid_out_ = Kp * error + I_mem * dt_ + Kd * (error - error_old) / dt_;

	error_old = error;

	if (pid_out_ > MAX_SPEED) pid_out_ = MAX_SPEED;
	else if (pid_out_ < -MAX_SPEED) pid_out_ = -MAX_SPEED;

	if (pid_out_ < 1 && pid_out_ > -1) pid_out_ = 0;

	return pid_out_;
}

float speedPIDControl(float _dt, float _input, float _setpoint)
{
	float alpha, beta, gama, error, out;

	error = _setpoint - _input;
	alpha = 2 * dt * Kp + Ki * dt * dt + 2 * Kd;
	beta = Ki * dt * dt - 4 * Kd - 2 * dt * Kp;
	gama = 2 * Kd;
	out = (alpha * error + beta * sLastError + gama * sSecondToLastError + 2 * dt * sLastPIDout) / (2 * dt);
	sLastPIDout = out;
	sSecondToLastError = sLastError;
	sLastError = error;

	out = constrain(out, -MAX_SPEED, MAX_SPEED);

	if (out < NOISE && out > -NOISE)
		out = 0;
	return out;
}

float directionPIDControl(float _dt, float _input, float _setpoint)
{
	float alpha, beta, gama, error, out;
	error = _setpoint - _input;
	alpha = 2 * dt * Kpd + Kid * dt * dt + 2 * Kdd;
	beta = Kid * dt * dt - 4 * Kdd - 2 * dt * Kpd;
	gama = 2 * Kdd;
	out = (alpha * error + beta * dLastError + gama * dSecondToLastError + 2 * dt * dlastPIDout) / (2 * dt);
	dlastPIDout = out;
	dSecondToLastError = dLastError;
	dLastError = error;

	out = constrain(out, -MAX_STEERING, MAX_STEERING);

	if (out < NOISE && out > -NOISE)
		out = 0;
	return out;
}

void SerialEvent()
{
	while (COMMAND_PORT.available())
	{
		char inChar = (char)COMMAND_PORT.read();

		if (inChar == '\n')
		{
			isStringCompleted = true;
			break;
		}

		inputString += inChar;
	}

	if (!isStringCompleted)
		return;

	String messageBuffer = inputString.substring(0, 2);

	if (messageBuffer == "KP")
	{
		Kp = inputString.substring(3).toFloat();
		COMMAND_PORT.println("Ok");
	}
	else if (messageBuffer == "KI")
	{
		Ki = inputString.substring(3).toFloat();
		COMMAND_PORT.println("Ok");
	}
	else if (messageBuffer == "KD")
	{
		Kd = inputString.substring(3).toFloat();
		COMMAND_PORT.println("Ok");
	}
	else if (messageBuffer == "PD")
	{
		Kpd = inputString.substring(3).toFloat();
		COMMAND_PORT.println("Ok");
	}
	else if (messageBuffer == "ID")
	{
		Kid = inputString.substring(3).toFloat();
		COMMAND_PORT.println("Ok");
	}
	else if (messageBuffer == "DD")
	{
		Kdd = inputString.substring(3).toFloat();
		COMMAND_PORT.println("Ok");
	}
	else if (messageBuffer == "SA")
	{
		float _sp = inputString.substring(3).toFloat();
		if (_sp == 0)
		{
			targetAngle = dmpGetTheta();
		}
		else
		{
			targetAngle = _sp;
		}
		COMMAND_PORT.println(targetAngle);
		COMMAND_PORT.println("Ok");
	}
	else if (messageBuffer == "SD")
	{
		float _sp = inputString.substring(3).toFloat();
		if (_sp == 0)
		{
			targetDir = dmpGetPsi();
		}
		else
		{
			targetDir = _sp;
		}
		COMMAND_PORT.println(targetDir);
		COMMAND_PORT.println("Ok");
	}
	inputString = "";
	isStringCompleted = false;
}

void setup()
{
	COMMAND_PORT.begin(115200);
	DEBUG_PORT.begin(115200);

	Kpd = Kid = Kpd = 0;
	pinMode(TRIG, OUTPUT);
	pinMode(ECHO, INPUT);
	WRITE(TRIG, 0);

	InitMPU();
	InitMotors();

	setLeftMotorSpeed(0);
	setRightMotorSpeed(0);

	oldMicros = micros();
	beginMicros = micros();
}

void loop()
{
	SerialEvent();

	currentMicros = micros();
	fifoCount = mpu.getFIFOCount();
	if (fifoCount >= 18)
	{
		if (fifoCount > 18)
		{
			mpu.resetFIFO();
			return;
		}

		dt = currentMicros - oldMicros;
		dt = dt * 0.001; //msec
		oldMicros = currentMicros;

		//currentAngle = 0.9 * currentAngle + 0.1 * dmpGetTheta();
		currentAngle = dmpGetTheta();
		currentDir = dmpGetPsi();
		currentDir = roundf(currentDir);

#if DEBUG == 1
		DEBUG_PORT.print("theta: ");
		DEBUG_PORT.println(currentAngle);
		DEBUG_PORT.print("psi: ");
		DEBUG_PORT.println(currentDir);
#endif

		speedOut = CalcuPID(dt, currentAngle, targetAngle);
		if (!isWaitingFor10Secs)
			dirOut = directionPIDControl(dt, currentDir, targetDir);

#if DEBUG == 2
		DEBUG_PORT.print("speedOut: ");
		DEBUG_PORT.println(speedOut);
		DEBUG_PORT.print("dirOut: ");
		DEBUG_PORT.println(dirOut);
#endif

		lMotorSpeed = speedOut + dirOut + steering;
		rMotorSpeed = speedOut - dirOut + steering;
#if DEBUG == 3		
		DEBUG_PORT.print("LeftSpeed: ");
		DEBUG_PORT.println(lMotorSpeed);
#endif
		setMotorsSpeed(lMotorSpeed, rMotorSpeed);

		if (isWaitingFor10Secs && (currentMicros - beginMicros > 10000000))
		{
			checkObstacleTimer = currentMicros;
			isWaitingFor10Secs = false;
			//targetAngle += 2;

			DEBUG_PORT.println("Start Moving!");
		}

		if (!isWaitingFor10Secs && currentMicros - checkObstacleTimer > 500000)
		{
			checkObstacleTimer = currentMicros;
			WRITE(TRIG, 1);
			delay_us(11);
			WRITE(TRIG, 0);
			duration = pulseIn(ECHO, HIGH, 5000);
			distance = duration * PULSE2DISTANCE;

			if (distance > 5 && distance < 30)
			{
				obstacleNum++;
				if (obstacleNum == 1)
				{
					targetDir += 90;
				}
				else if (obstacleNum == 2)
				{
					targetAngle = 0;
					steering = 0;
				}
			}
		}
	}
}