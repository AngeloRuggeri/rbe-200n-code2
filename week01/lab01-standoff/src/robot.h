#include <Chassis.h>

#include "standoff.h"

//TODO: You'll want to make a wall_follower class to mimic the standoff
//#include "wall_follower.h" 

#define IR_PIN 15

class Robot
{
protected:
    //WallFollower wallFollower;
    StandoffController standoffController;

    enum ROBOT_STATE {ROBOT_IDLE, ROBOT_WALL_FOLLOWING, ROBOT_STANDOFF};
    ROBOT_STATE robotState = ROBOT_IDLE;

public:
    Robot(void);
    void init(void);
    void loop(void);

protected:
    void handleIRPress(int16_t);
    void handleNewDistanceReading(float);
};