#include <Arduino.h>
#include "MPin.h"
#include "MMPU6050.h"
#include "Step.h"

#define DEBUG 4
#define PID_TUNING_MODE

//angle to speed
#define KP 0.55  //0.2
#define KD 0.16 //26

//desired speed to target angle
#define KPD 0.015//0.0125//0.015 //0.01	//0.065
#define KID 0.00007	 //0.00006 //0.05

#define NOISE 0.45 //+- mm/s

#define ANGLE_OFFSET 3.2 //0.08
#define MAX_TARGET_ANGLE 12
#define MAX_PD_OUT 700

#define STEERING 40
#define MOVING_SPEED 150

#define DEBUG_PORT Serial
#define COMMAND_PORT Serial1

bool isStringCompleted = false;
String inputString;

uint32_t oldMicros;
uint32_t currentMicros;
float dt;

float Kp = KP, Kd = KD;
float desiredSpeed, currentAngle, oldAngle, angleOffset = ANGLE_OFFSET;

float Kpd = 0, Kid = 0;
float dirOut = 0, currentDir;

float targetAngle = 0;
float targetDir = 0;

float lMotorSpeed, rMotorSpeed;
float steering = 0;

float PDout;
bool isRotating = false;
bool isWaitingFor10Secs = true;
uint32_t beginMicros;

uint32_t checkObstacleTimer;
uint32_t nextObstacleDelay;
uint32_t delayAfterRotate, delayBeforeRotate;
bool waitingToBalacingState = false;
uint8_t obstacleNum = 0;

uint16_t actualRobotSpeed, oldActualRobotSpeed;
float estimatedSpeedFiltered;

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
	else if (messageBuffer == "SA")
	{
		float _sp = inputString.substring(3).toFloat();
		if (_sp == 0)
		{
			targetAngle = currentAngle;
		}
		else
		{
			targetAngle = _sp;
		}
		COMMAND_PORT.println(targetAngle);
		COMMAND_PORT.println("Ok");
	}
	else if (messageBuffer == "DS")
	{
		desiredSpeed = inputString.substring(3).toFloat();
		COMMAND_PORT.println("Ok");
	}
	else if (messageBuffer == "ST")
	{
		steering = inputString.substring(3).toFloat();
		COMMAND_PORT.println("Ok");
	}
	else if (messageBuffer == "AO")
	{
		angleOffset = inputString.substring(3).toFloat();
		COMMAND_PORT.println("Ok");
	}
	inputString = "";
	isStringCompleted = false;
}

void setup()
{
	COMMAND_PORT.begin(115200);
	DEBUG_PORT.begin(115200);

	pinMode(SENSOR, INPUT);

	InitMPU();
	InitMotors();

	setMotorsSpeed(0, 0);
	desiredSpeed = 0;
	oldMicros = micros();
	beginMicros = micros();
	COMMAND_PORT.println("Begin");
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
		dt = dt * 0.000001f; //sec
		oldMicros = currentMicros;

		oldAngle = currentAngle;
		currentAngle = dmpGetTheta() - angleOffset;
		//currentAngle = 0.99 * currentAngle + 0.01 * (dmpGetTheta() - angleOffset);

		currentDir = dmpGetPsi();
		currentDir = roundf(currentDir);

		//COMMAND_PORT.println(currentAngle);

#if DEBUG == 1
		DEBUG_PORT.print("theta: ");
		DEBUG_PORT.println(currentAngle);
#endif

		oldActualRobotSpeed = actualRobotSpeed;
		actualRobotSpeed = (leftMotorSpeed + rightMotorSpeed) / 2;

		int16_t angularVelo = (currentAngle - oldAngle) * 25; //?????
		int16_t estimatedSpeed = -oldActualRobotSpeed + angularVelo;
		estimatedSpeedFiltered = estimatedSpeed * 0.9 + (float)estimatedSpeed * 0.1;

		targetAngle = speedPIControl(dt, estimatedSpeedFiltered, desiredSpeed, Kpd, Kid);
		targetAngle = constrain(targetAngle, -MAX_TARGET_ANGLE, MAX_TARGET_ANGLE);

#if DEBUG == 2
		DEBUG_PORT.print("currentAngle: ");
		DEBUG_PORT.println(currentAngle);
		DEBUG_PORT.print("targetAngle: ");
		DEBUG_PORT.println(targetAngle);
#endif

		PDout += stabilityPDControl(dt, currentAngle, targetAngle, Kp, Kd);
		PDout = constrain(PDout, -MAX_PD_OUT, MAX_PD_OUT);

		if (PDout > -NOISE && PDout < NOISE)
			PDout = 0;
#if DEBUG == 3
		DEBUG_PORT.print("PDout: ");
		DEBUG_PORT.println(PDout);
#endif

		lMotorSpeed = PDout - steering;
		rMotorSpeed = PDout + steering;
	}

	if (currentAngle < 40 && currentAngle > -40)
	{
		setMotorsSpeed(lMotorSpeed, rMotorSpeed);
	}
	else
	{
		setMotorsSpeed(0, 0);
		PID_errorSum = 0;
	}

#ifndef PID_TUNING_MODE

	if (isWaitingFor10Secs && (currentMicros - beginMicros > 10000000))
	{
		Kpd = KPD;
		Kid = KID;
		checkObstacleTimer = currentMicros;
		isWaitingFor10Secs = false;
		desiredSpeed = -MOVING_SPEED;

		COMMAND_PORT.println("Start Moving!");
		delayBeforeRotate = currentMicros;
	}
	if (currentDir > targetDir - 3 && currentDir < targetDir + 3)
	{
		isRotating = false;
		steering = 0;
		desiredSpeed = -MOVING_SPEED;
		//delayAfterRotate = currentMicros;
		//waitingToBalacingState = true;
	}
	// if (currentMicros - delayAfterRotate > 5000)
	// {
	// 	if (waitingToBalacingState)
	// 	{
	// 		waitingToBalacingState = false;
	// 		desiredSpeed = -MOVING_SPEED;
	// 	}
	// }
	if (!isWaitingFor10Secs && (currentMicros - checkObstacleTimer > 100000))
	{
		checkObstacleTimer = currentMicros;
		if (currentMicros - delayBeforeRotate > 4000000)
		{
			nextObstacleDelay = currentMicros;

			if (isRotating)
				return;

			obstacleNum++;

			COMMAND_PORT.println("Detected Obstacle!");
			if (obstacleNum == 1)
			{
				isRotating = true;
				steering = STEERING;
				desiredSpeed = 0;
				targetDir = currentDir - 180;
				if (targetDir < -180)
					targetDir += 360;
			}
			else if (obstacleNum == 2)
			{
				//Kid = 0;
				//Kpd = 0;
				steering = 0;
				desiredSpeed = 0;
				//COMMAND_PORT.println("Stop Moving!");
			}
		}
	}
#endif
}