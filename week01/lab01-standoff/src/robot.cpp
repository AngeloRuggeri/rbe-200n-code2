#include "robot.h"

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
    chassis.init();

    irDecoder.init(IR_PIN);
    mb_ez1.init(USE_ECHO);  // TODO: use the sensor/method of your choice
}

void Robot::loop() 
{
    //check the IR remote
    int16_t keyCode = irDecoder.getKeyCode();
    if(keyCode != -1) handleIRPress(keyCode);

    //check the distance sensor
    float distanceReading = 0;
    bool hasNewReading = mb_ez1.getDistance(distanceReading);
    if(hasNewReading) handleNewDistanceReading(distanceReading);
}

void Robot::handleIRPress(int16_t key)
{
    Serial.println(key);
    if(key == MUTE)
    {
        chassis.stop();
        robotState = ROBOT_IDLE;
        return;
    }

    switch(robotState)
    {
        case ROBOT_IDLE:
            if(key == BACK)
            {
                robotState = ROBOT_STANDOFF;
            }
            if(key == PREV)
            {
                robotState = ROBOT_WALL_FOLLOWING;
            }
            break;
        case ROBOT_STANDOFF:
            standoffController.handleKeyPress(key);
            break;
        //TODO: Add case for wall following
        default:
            break;
    }
}

void Robot::handleNewDistanceReading(float distanceReading)
{
    //comment out after you verify this works
    Serial.print(millis());
    Serial.print('\t');
    Serial.print(distanceReading);
    Serial.print('\t');
    
    //TODO: Add wall following behaviour

    if(robotState == ROBOT_STANDOFF)
    {
        standoffController.processDistanceReading(distanceReading);
        chassis.setMotorEfforts(standoffController.leftEffort, standoffController.rightEffort);
    }   

    Serial.print('\n');
}