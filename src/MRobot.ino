#include <Arduino.h>
#include "MPin.h"
#include "MEEPROM.h"
#include "MMPU6050.h"
#include "Step.h"

#define KP 50
#define KI 1.5
#define KD 0.08
#define NOISE 1.5 //+-2mm/s
#define COMMAND_PORT Serial1

bool isStringCompleted = false;
String inputString;

uint32_t oldMicros;
uint32_t currentMicros;
float dt, error, error_old, I_mem;
float lastError, secondToLastError;
float alpha, beta, gama;
float Kp = KP, Ki = KI, Kd = KD;
float PIDout = 0, lastPIDout = 0;
float steering = 0;
float setpoint = -3.1;

float angle_adjusted;
float angle_adjusted_Old;

bool BATlow = false;

float y, p, r;

float CalcuPID(float dt_, float error_, float set_point_)
{
	float pid_out_;

	error = error_ - set_point_;
	I_mem += error * Ki;
	if (I_mem > MAX_SPEED)
		I_mem = MAX_SPEED;
	else if (I_mem < -MAX_SPEED)
		I_mem = -MAX_SPEED;
	pid_out_ = Kp * error + I_mem * dt_ + Kd * (error - error_old) / dt_;

	error_old = error;

	if (pid_out_ > MAX_SPEED)
		pid_out_ = MAX_SPEED;
	else if (pid_out_ < -MAX_SPEED)
		pid_out_ = -MAX_SPEED;

	if (pid_out_ < NOISE && pid_out_ > -NOISE)
		pid_out_ = 0;

	PIDout = pid_out_;
	return pid_out_;
}

float PID(float _dt, float _feedback, float _setpoint)
{
	error = _setpoint - _feedback;
	alpha = 2 * dt * Kp + Ki * dt * dt + 2 * Kd;
	beta = Ki * dt * dt - 4 * Kd - 2 * dt * Kp;
	gama = 2 * Kd;
	PIDout = (alpha * error + beta * lastError + gama * secondToLastError + 2 * dt * lastPIDout) / (2 * dt);
	lastPIDout = PIDout;
	secondToLastError = lastError;
	lastError = error;

	if (PIDout > MAX_SPEED)
		PIDout = MAX_SPEED;
	else if (PIDout < -MAX_SPEED)
		PIDout = -MAX_SPEED;

	if (PIDout < NOISE && PIDout > -NOISE)
		PIDout = 0;
	return PIDout;
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
	else if (messageBuffer == "SP")
	{
		float _sp = inputString.substring(3).toFloat();
		if (_sp == 0)
		{
			ReadGyroValue(&y, &p, &r);
			setpoint = p;
			COMMAND_PORT.println(p);
			COMMAND_PORT.println("Ok");
		}
		else
		{
			setpoint = _sp;
			COMMAND_PORT.println("Ok");
		}
	}
	else if (messageBuffer == "SI")
	{
		setpoint += 0.01;
		COMMAND_PORT.println(setpoint);
		COMMAND_PORT.println("Ok");
	}
	else if (messageBuffer == "SD")
	{
		setpoint -= 0.01;
		COMMAND_PORT.println(setpoint);
		COMMAND_PORT.println("Ok");
	}
	else if (messageBuffer == "SA")
	{
		SaveData(&Kp, &Ki, &Kd);
		COMMAND_PORT.println("Ok");
	}
	inputString = "";
	isStringCompleted = false;
}

void setup()
{
	COMMAND_PORT.begin(115200);
	Serial.begin(115200);
	// if (!InitEEPROM(&Kp, &Ki, &Kd))
	// {
	// 	Kp = KP;
	// 	Ki = KI;
	// 	Kd = KD;
	// }
	InitMPU();
	InitMotors();

	setLeftMotorSpeed(0);
	setRightMotorSpeed(0);

	oldMicros = micros();
}

void loop()
{
	if (ReadGyroValue(&y, &p, &r) != -1)
	{
		currentMicros = micros();
		dt = currentMicros - oldMicros;
		dt = dt * 0.001; //msec
		oldMicros = currentMicros;

		if (p > 45 || p < -45)
		{
			steering = 0;
		}
		else
		{
			PID(dt, p, setpoint);
			steering = PIDout;
		}
	}

	setMotorsSpeed(steering, steering);
	SerialEvent();
}