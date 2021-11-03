#include "robot.h"

// NOTE THAT MY IR KEYS AND CODES ARE DIFFERENT FROM YOURS!!! Add/adust as needed
#include "ir_codes.h"
#define IR_PIN 15

#define HC_TRIG 16       //for pinging -- not a great choice since this can hamper uploading
#define HC_ECHO 17    //for reading pulse

#define MB_CTRL 2
#define MB_PULSE 35

Robot::Robot(void)
    : filter(5), hcsr04(HC_ECHO, HC_TRIG), irDecoder(IR_PIN)
{
    //nothing to see here; move along...
}

void Robot::init(void)
{
    chassis.init();

    irDecoder.init();
    hcsr04.init(USE_ECHO | USE_CTRL_PIN);  // TODO: use the sensor/method of your choice
}

void Robot::loop() 
{
    //check the IR remote
    int16_t keyCode = irDecoder.getKeyCode();
    if(keyCode != -1) handleIRPress(keyCode);

    //check if time to send ping
    //hcsr04.checkPingTimer();

    /** Check the distance sensor.
     * We return true only if there is a new reading, which is passed by reference.
     * It hardly needs to be done this way, but passing by reference is a useful tool,
     * so we start with a 'lightweight' example here.
     */
    float distanceReading = 0;
    bool hasNewReading = hcsr04.getDistance(distanceReading);
    if(hasNewReading) handleNewDistanceReading(distanceReading);

    /**
     * Here we check if the motors/pose were recently updated.
     * If so, then call handleUpdateReady(), which does additional tasks
     */
    if(chassis.readyForUpdate) handleUpdateReady();
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
            if(key == UP)
            {
                robotState = ROBOT_DRIVE;
                chassis.setTwist(10, 0);
            }
            //TODO: add other keys
            break;

        case ROBOT_STANDOFF:
            standoffController.handleKeyPress(key);
            break;
        //TODO: Add case for wall following
        default:
            break;
    }
}

int count = 0;

void Robot::handleNewDistanceReading(float distanceReading)
{
    //comment out after you verify this works
    // Serial.print(millis());
    // Serial.print('\t');
    // Serial.print(distanceReading);
    // Serial.print('\t');

    float averageDist = filter.addAndReturnMedian(distanceReading);
    // Serial.print(averageDist);
    // Serial.print('\t');

    if(robotState == ROBOT_WALL_FOLLOWING)
    {    
        //TODO: Add wall following behaviour
    }

    if(robotState == ROBOT_STANDOFF)
    {
        standoffController.processDistanceReading(averageDist);
        chassis.setMotorEfforts(standoffController.leftEffort, standoffController.rightEffort);
    }   

    if(robotState == ROBOT_DRIVE) //if we're driving and we see an object that's too close, stop
    {
        //TODO: check if there is an object too close and emergency stop
    }
}

/**
 * This gets called whenever the drive motors and pose get updated.
 */
void Robot::handleUpdateReady(void)
{
    chassis.readyForUpdate = false;

    Serial.print(millis());
    Serial.print('\t');

    chassis.writePose();

    if(robotState == ROBOT_DRIVE_TO_POINT)
    {
        //TODO: calculate motor inputs to give desired motion

        //TODO: check if we've reached destination
    }

    Serial.print('\n');
}
