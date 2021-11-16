#include "Chassis.h"
#include <math.h>

/**
 * Chassis class
 */

Chassis::Chassis(void) : leftMotor(MOTOR_LEFT_PWM, MOTOR_LEFT_DIR, MOTOR_LEFT_ENCA, MOTOR_LEFT_ENCB),
						 rightMotor(MOTOR_RIGHT_PWM, MOTOR_RIGHT_DIR, MOTOR_RIGHT_ENCA, MOTOR_RIGHT_ENCB)
{
	//TODO: Set gains for speed control
	//Either click into the MotorEncoded class and change the initializer in the
	//constructor, or manually set the gains with the setter method: setPIDgains(...)
}

void Chassis::init(void)
{
	allocateTimer(); // used by the DC Motors
	destPose.x = -10000;
	leftMotor.attach();
	rightMotor.attach();

	stop();
}

void Chassis::setWheelSpeeds(float left, float right)
{
	leftMotor.setTargetDegreesPerSecond(left);
	rightMotor.setTargetDegreesPerSecond(right);
}

void Chassis::setMotorEfforts(float left, float right)
{
	leftMotor.setEffortPercent(left);
	rightMotor.setEffortPercent(right);
}

//speed in cm/sec; omega in rad/sec
void Chassis::setTwist(float speed, float omega)
{
	//TODO: calculate desired motor speeds for given motion
	float leftSp = 0;
	float rightSp = 0;

	Serial.print(leftSp);
	Serial.print('\t');
	Serial.print(rightSp);
	Serial.print('\t');

	setWheelSpeeds(leftSp, rightSp);
}

void Chassis::updatePose(float leftDelta, float rightDelta) //parameters in degrees of wheel rotation
{

	float lD = leftDelta * PI / 180.00 * 0.07 / 2;
	float rD = rightDelta * PI / 180.00 * 0.07 / 2;

	float newPose = currPose.theta + (rD - lD) / (0.14 / 2);
	currPose.theta = (currPose.theta + newPose) / 2;
	currPose.x = (currPose.x + (lD + rD) / 2 * cos(currPose.theta));
	currPose.y = (currPose.y + (lD + rD) / 2 * sin(currPose.theta));
}

void Chassis::writePose(void)
{
	Serial.print(currPose.x * -100.00);
	Serial.print('\t');
	Serial.print(currPose.y * -100.00);
	Serial.print('\t');
	Serial.print(currPose.theta * (180 / PI));
	Serial.print('\t');
}

void Chassis::driveToPoint(int p)
{

	updatePose(leftMotor.getDelta(), rightMotor.getDelta());

	if (p == 1)
	{
		destPose.x = -0.30;
		destPose.y = -0.30;
	}

	if (p == 2)
	{
		destPose.x = -0.60;
		destPose.y = 0;
	}

	if (p == 3)
	{
		destPose.x = -0.30;
		destPose.y = 0.30;
	}

	if (p == 0)
	{
		destPose.x = 0;
		destPose.y = 0;
	}

	destPose.theta = atan((destPose.y - currPose.y) / (destPose.x - currPose.x));
	float deltaDist = (-destPose.theta - currPose.theta) * 0.14 / 2;
	float sumDist = -2 * (destPose.x - currPose.x) / cos(destPose.theta);
	float rightDist = (deltaDist + sumDist) / 2;
	float leftDist = (sumDist - rightDist);
	float rightDeg = rightDist * (2 / 0.07) * (180 / PI);
	float leftDeg = leftDist * (2 / 0.07) * (180 / PI);
	float rightEffort = rightDeg / 35;
	float leftEffort = leftDeg / 35;

	setMotorEfforts(-leftEffort + 1, -rightEffort);

	//TODO: you'll need to add PID control here, both for distance and heading!
}

bool Chassis::checkDestination(void)
{

	//currPose.x + 0.04 < destPose.x && currPose.x - 0.04 > destPose.x && currPose.y + 0.04 < destPose.y && currPose.y - 0.04 > destPose.y && currPose.theta + 6 < destPose.theta && currPose.theta - 6 > destPose.theta

	if (currPose.x + 0.05 > destPose.x && currPose.x - 0.05 < destPose.x
	&& currPose.y + 0.05 > destPose.y && currPose.y - 0.05 < destPose.y)

	{
		return true;
	}
	//TODO: Check if you've reached destination
	return false;
}

void Chassis::motorHandler(void)
{
	if (!timerAllocated)
		allocateTimer();

	if (++velocityLoopCounter == controlIntervalMS)
	{
		velocityLoopCounter = 0;

		leftMotor.process();
		rightMotor.process();

		//here's where you'll update the pose in Lab 3, née 2
		updatePose(leftMotor.getDelta(), rightMotor.getDelta());

		Serial.println("");
		Serial.println("");
		Serial.println("");
		Serial.println("CURR POSE");

		writePose();

		Serial.println("");
		Serial.println("");
		Serial.println("");
		Serial.println("DEST POSE");

		Serial.print(destPose.x * -100.00);
		Serial.print('\t');
		Serial.print(destPose.y * -100.00);
		Serial.print('\t');
		Serial.print(destPose.theta * (180 / PI));
		Serial.print('\t');
		//		writePose();

		readyForUpdate = true;
	}
}

static TaskHandle_t complexHandlerTask;

void onMotorTimer(void *param)
{
	ESP_LOGI(TAG, "Starting the PID loop thread");
	TickType_t xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();
	const TickType_t xInterval = 1; // sets up 1ms timer
	while (true)
	{
		vTaskDelayUntil(&xLastWakeTime, xInterval);
		Chassis *pChassis = (Chassis *)param;
		if (pChassis)
			pChassis->motorHandler();
		else
			Serial.println("NULL pointer in onMotorTimer()!!");
	}
	ESP_LOGE(TAG, "ERROR PID thread died!");
}

void Chassis::allocateTimer(void)
{
	if (!timerAllocated)
	{
		xTaskCreatePinnedToCore(onMotorTimer, "PID loop Thread", 8192, this, 1, &complexHandlerTask, 0);
	}

	timerAllocated = true;
}
