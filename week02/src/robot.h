#include <Chassis.h>
#include <Rangefinder.h>
#include <IRdecoder.h>

#include "standoff.h"

//TODO: You'll want to make a wall_follower class to mimic the standoff
//#include "wall_follower.h" 

class Robot
{
protected:
    Chassis chassis;
    
    //WallFollower wallFollower;
    StandoffController standoffController;
    AveragingFilter filter;

    Rangefinder hcsr04;
    IRDecoder irDecoder;
    
    enum ROBOT_STATE{   
                        ROBOT_IDLE, 
                        ROBOT_WALL_FOLLOWING, 
                        ROBOT_STANDOFF, 
                        ROBOT_DRIVE, 
                        ROBOT_DRIVE_TO_POINT
                    };

    ROBOT_STATE robotState = ROBOT_IDLE;

public:
    Robot(void);
    void init(void);
    void loop(void);

protected:
    void handleIRPress(int16_t);
    void handleNewDistanceReading(float);
    void handleUpdateReady(void);
};