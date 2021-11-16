#pragma once

#include <RBE-200n-Lib.h>
#include <ESP32Servo.h>
#include <MotorEncoded.h>
#include "pose.h"

/**
 * Chassis class
 */

#define WHEEL_TRACK	14.0 //cm
#define WHEEL_RADIUS 3.5 //cm

class Chassis
{
protected:
	/**
	 * Loop rate for this motor. The main timer loop is 1ms, so this value is in ms.
	 */
	uint32_t controlIntervalMS = DRIVE_MOTOR_CTRL_INT_MS;

	/**
	 * An internal counter that counts iterations of the main motor loop; it is compared to
	 * controlIntervalMS so that the velocity PID runs on longer intervals.
	 */
	uint32_t velocityLoopCounter = 0;

    MotorEncoded leftMotor;
    MotorEncoded rightMotor;

    bool timerAllocated = false;
    void allocateTimer(void); 
    void motorHandler(void);
    friend void onMotorTimer(void* param);
    
    float wheelRadius = WHEEL_RADIUS;
    float wheelTrack = WHEEL_TRACK;

    Pose currPose;
    Pose destPose;

public:
    bool readyForUpdate = false;
    
public:
    Chassis(void);

    void init(void);
    void loop(void);

    void stop(void) {setMotorEfforts(0,0);}

    void setWheelSpeeds(float left, float right);
    void setMotorEfforts(float left, float right);

    void setTwist(float u, float omega);    //implementation left to the student
    void updatePose(float, float);          //implementation left to the student

    void writePose(void);

    void driveToPoint(int);                //implementation left to the student
    bool checkDestination(void);

};
