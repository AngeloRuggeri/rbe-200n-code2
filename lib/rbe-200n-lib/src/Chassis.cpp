#include "Chassis.h"
#include <math.h>

/**
 * Chassis class
 */

Chassis::Chassis(void) :
    leftMotor(MOTOR_LEFT_PWM, MOTOR_LEFT_DIR, MOTOR_LEFT_ENCA, MOTOR_LEFT_ENCB),
    rightMotor(MOTOR_RIGHT_PWM, MOTOR_RIGHT_DIR, MOTOR_RIGHT_ENCA, MOTOR_RIGHT_ENCB)
{
    //TODO: Set gains for speed control
	//Either click into the MotorEncoded class and change the initializer in the 
	//constructor, or manually set the gains with the setter method: setPIDgains(...)
}

void Chassis::init(void)
{
	allocateTimer(); // used by the DC Motors

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

void Chassis::updatePose(float leftDelta, float rightDelta, float deltaT) //parameters in degrees of wheel rotation
{

	float R = 0.14/2 * (rightDelta + leftDelta)/(rightDelta-leftDelta);
	float D = ((leftDelta + rightDelta)/2) * 0.07 * PI * deltaT/1000 * 0.166666666666667 / 60;
	float phi = D/R;
	currPose.x = R * sin(phi) + currPose.x;
	currPose.y = R- R * cos(phi) + currPose.y;
	currPose.theta = phi - currPose.theta;
	//TODO: update pose from actual wheel motion
}

void Chassis::writePose(void)
{
	Serial.print(currPose.x);
	Serial.print('\t');
	Serial.print(currPose.y);
	Serial.print('\t');
	Serial.print(currPose.theta);
	Serial.print('\t');
}

void Chassis::driveToPoint(void) 
{
	//TODO: you'll need to add PID control here, both for distance and heading!
}

bool Chassis::checkDestination(void) 
{
	//TODO: Check if you've reached destination
	return false;
}

void Chassis::motorHandler(void)
{
    if(!timerAllocated) allocateTimer();

	if(++velocityLoopCounter == controlIntervalMS) 
	{
		velocityLoopCounter = 0;

		leftMotor.process();
		rightMotor.process();

		//here's where you'll update the pose in Lab 3, née 2
	//	updatePose(leftMotor.getDelta(), rightMotor.getDelta());

		readyForUpdate = true;
	}
}

static TaskHandle_t complexHandlerTask;

void onMotorTimer(void* param)
{
	ESP_LOGI(TAG, "Starting the PID loop thread");
	TickType_t xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();
    const TickType_t xInterval = 1; 	// sets up 1ms timer
	while(true)
	{
		vTaskDelayUntil(&xLastWakeTime, xInterval);
		Chassis* pChassis = (Chassis*) param;
		if(pChassis) pChassis->motorHandler();
		else Serial.println("NULL pointer in onMotorTimer()!!");
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
