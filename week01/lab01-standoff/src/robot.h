#include <Chassis.h>
#include <IRdecoder.h>

#include "standoff.h"
#include "wallFollow.h"

//TODO: You'll want to make a wall_follower class to mimic the standoff
//#include "wall_follower.h" 

#define IR_PIN 15

class Robot
{
protected:
    WallFollowController wallFollowController;
    StandoffController standoffController;

    enum ROBOT_STATE {ROBOT_IDLE, ROBOT_WALL_FOLLOWING, ROBOT_STANDOFF, DRIVE_STRAIGHT, SPIN_CCW, ARC_90, DRIVE_TO_POINT};
    ROBOT_STATE robotState = ROBOT_IDLE;
    float gyroBias = 0;

public:
    Robot(void);
    void init(void);
    void loop(void);

protected:
    void handleIRPress(int16_t);
    void handleNewDistanceReading(float);

    Chassis chassis;
    IRDecoder irDecoder = IRDecoder(15);
    void handleChassisUpdate(void);
    void handleIMUtimer();
};