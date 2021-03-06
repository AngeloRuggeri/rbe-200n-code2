#include <PIDcontroller.h>

class WallFollowController
{
public:
    float leftEffort = 0;
    float rightEffort = 0;

protected:
    float targetDistance = 40;

    PIDController piWallFollow;

public:
    WallFollowController(void) : piWallFollow(0.05, 0.01, 0) {} //TODO: edit gains

    void processDistanceReading(float distance);
    void handleKeyPress(int16_t key);
};