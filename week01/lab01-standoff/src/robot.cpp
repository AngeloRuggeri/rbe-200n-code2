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
    irDecoder.init(IR_PIN);
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
}

void Robot::handleIRPress(int16_t key)
{

    Serial.print(key);

    if (key == NUM_2)
    {
        chassis.stop();
        Serial.println("NUM 2");
        robotState = ROBOT_IDLE;
        return;
    }

    switch (robotState)
    {
    case ROBOT_IDLE:
        if (key == NUM_4)
        {
            robotState = ROBOT_STANDOFF;
            Serial.println("NUM 4");
        }
        if (key == NUM_3)
        {
            robotState = ROBOT_WALL_FOLLOWING;
            Serial.println("NUM 3");
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
        chassis.setWheelSpeeds(180, 180);
        break;

    case SPIN_CCW:
        chassis.setWheelSpeeds(180, -180);
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