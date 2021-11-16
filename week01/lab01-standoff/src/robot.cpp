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

//     if (chassis.checkDestination())
//     {
//         chassis.setMotorEfforts(0, 0);
//  //       Serial.println("AT DESTINATION");
//     }



    if (chassis.readyForUpdate)
        handleChassisUpdate();
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
            chassis.setDestination(0, 0);
            Serial.println("Drive to point");
        }

        if (key == NUM_1)
        {
            robotState = DRIVE_TO_POINT;
            chassis.setDestination(0.3, 0.3);
            Serial.println("Drive to point");
        }

        if (key == NUM_2)
        {
            robotState = DRIVE_TO_POINT;
            chassis.setDestination(0.6, 0);
            Serial.println("Drive to point");
        }

        if (key == NUM_3)
        {
            robotState = DRIVE_TO_POINT;
            chassis.setDestination(0.3, -0.3);
            Serial.println("Drive to point");
        }

        if (key == NUM_6)
        {
            robotState = ARC_90;
            Serial.println("NUM 6");
        }

        if (key == NUM_7)
        {
            robotState = ROBOT_WALL_FOLLOWING;
            Serial.println("NUM 7");
        }
        if (key == NUM_8)
        {
            robotState = ROBOT_STANDOFF;
            Serial.println("NUM 8");
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

void Robot::handleChassisUpdate(void)
{
    chassis.writePose();
    chassis.readyForUpdate = 0;
    if (robotState == DRIVE_TO_POINT)
    {
        chassis.driveToPoint();
    }
    Serial.print('\n');
}