#include "robot.h"
#include "Chassis.h"

// NOTE THAT MY IR KEYS AND CODES ARE DIFFERENT FROM YOURS!!! Add/adust as needed
#include "ir_codes.h"

#include <maxbotix.h>
#include <IRdecoder.h>

Robot::Robot(void)
{
    //nothing to see here; move along...
}

void Robot::init(void)
{
    delay(1000);
    Serial.begin(115200);
    chassis.init();
    mb_ez1.init();
    irDecoder.init();
    //   mb_ez1.init(USE_ECHO);  // TODO: use the sensor/method of your choice
}

void Robot::loop()
{
    //check the IR remote
    int16_t keyCode = irDecoder.getKeyCode();
    if (keyCode != -1)
        handleIRPress(keyCode);

    float distance = 0;

    bool newReading = mb_ez1.getDistance(distance);

    if (newReading)
        handleNewDistanceReading(distance);

    if (chassis.checkDestination())
    {
        chassis.setMotorEfforts(0, 0);
        Serial.println("AT DESTINATION");
    }
}

void Robot::handleIRPress(int16_t key)
{

    //   Serial.print(key);

    if (key == STOP)
    {
        chassis.stop();
        chassis.writePose();
        Serial.println("STOP");
        robotState = ROBOT_IDLE;
        return;
    }

    switch (robotState)
    {
    case ROBOT_IDLE:

        if (key == NUM_0)
        {
            robotState = DRIVE_TO_POINT;
            Serial.println("Drive to point");
        }

        if (key == NUM_1)
        {
            robotState = ARC_90;
            Serial.println("NUM 1");
        }

        if (key == NUM_3)
        {
            robotState = ROBOT_WALL_FOLLOWING;
            Serial.println("NUM 3");
        }
        if (key == NUM_4)
        {
            robotState = ROBOT_STANDOFF;
            Serial.println("NUM 4");
        }

        if (key == PREV)
        {
            robotState = DRIVE_STRAIGHT;
            Serial.print("UP Button");
        }

        if (key == LEFT)
        {
            robotState = SPIN_CCW;
            Serial.print("LEFT Button");
        }

        break;

    case ROBOT_STANDOFF:
        standoffController.handleKeyPress(key);
        break;

    case ROBOT_WALL_FOLLOWING:
        wallFollowController.handleKeyPress(key);
        break;

    case DRIVE_STRAIGHT:
        chassis.setWheelSpeeds(-180, -180);
        break;

    case SPIN_CCW:
        chassis.setWheelSpeeds(-180, 180);
        break;

    case ARC_90:
        chassis.setWheelSpeeds(-115, -90);
        break;

    case DRIVE_TO_POINT:

        if (key == NUM_1)
        {
            chassis.driveToPoint(1);
            Serial.println("30, 30");
        }
        if (key == NUM_2)
        {
            chassis.driveToPoint(2);
            Serial.println("60, 0");
        }

        if (key == NUM_3)
        {
            chassis.driveToPoint(3);
            Serial.println("30, -30");
        }

        if (key == NUM_0)
        {
            chassis.driveToPoint(0);
            Serial.println("0, 0");
        }
        break;

    default:
        break;
    }
}

void Robot::handleNewDistanceReading(float distanceReading)
{
    //comment out after you verify this works
    // Serial.print(millis());
    // Serial.print('\t');
    //  Serial.println(distanceReading);

    if (robotState == ROBOT_STANDOFF)
    {
        standoffController.processDistanceReading(distanceReading);
        chassis.setMotorEfforts(standoffController.leftEffort, standoffController.rightEffort);
    }

    if (robotState == ROBOT_WALL_FOLLOWING)
    {

        wallFollowController.processDistanceReading(distanceReading);
        chassis.setMotorEfforts(wallFollowController.leftEffort, wallFollowController.rightEffort);
    }
}