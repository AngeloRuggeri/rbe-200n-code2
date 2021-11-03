#include "wallFollow.h"
#include "ir_codes.h"

void WallFollowController::processDistanceReading(float distance)
{
    float error = distance - targetDistance ;
    float effort = piWallFollow.ComputeEffort(error);

    leftEffort = effort;
    rightEffort = effort;

    // Serial.print(targetDistance);
    // Serial.print('\t');
    // Serial.print(distance);
    // Serial.print('\t');
    // Serial.print(error);
    // Serial.print('\t');
    // Serial.print(effort);
    // Serial.print('\t');
    // Serial.print(leftEffort);
    // Serial.print('\t');
    // Serial.print(rightEffort);
    // Serial.print('\t');
}

void WallFollowController::handleKeyPress(int16_t key)
{
    switch(key)
    {
        case PREV:
            targetDistance += 10;
            break;

        case NEXT:
            targetDistance -= 10;
            break;

        default:
            if(key >= NUM_0 && key <= NUM_9)
            {
                
                //TODO: implement wallFollow code
            }
            break;
    }
}