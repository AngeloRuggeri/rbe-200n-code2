#include "wallFollow.h"
#include "ir_codes.h"

void WallFollowController::processDistanceReading(float distance)
{

    float baseSpeed = 20;
    float error = targetDistance - distance;
    float effort = piWallFollow.ComputeEffort(error);


    Serial.print(distance);

    Serial.print('\t');

    Serial.print(targetDistance);

    Serial.print('\t');

    Serial.print(error);

    Serial.println("");

    leftEffort = baseSpeed - effort;
    rightEffort = baseSpeed + effort;

//     if (error >= 5)
//     {
//         leftEffort = (effort / 1.5);
//         rightEffort = (effort);
//         // Serial.println("RIGHT");
//         // Serial.print('\t');
//         // Serial.print(leftEffort);
//         // Serial.print('\t');
//         // Serial.print(rightEffort);
//         // Serial.println("");
//     }

//     if (error <= -5)
//     {
//         leftEffort = -effort;
//         rightEffort = -(effort / 1.5);
//         // Serial.println("LEFT");
//         // Serial.print('\t');
//         // Serial.print(leftEffort);
//         // Serial.print('\t');
//         // Serial.print(rightEffort);
//         // Serial.println("");
//     }

//     else if (error > -5 && error < 5)
//     {
// //        Serial.println("AHHHHHHH");
//         leftEffort = baseSpeed;
//         rightEffort = baseSpeed;
//     }
}

void WallFollowController::handleKeyPress(int16_t key)
{
    switch (key)
    {

    case STOP:
        Serial.println("RESET");
        targetDistance = 40;
        break;

    case PREV:
        Serial.println("PREV");
        leftEffort += 10;
        rightEffort += 10;

        Serial.print(leftEffort);
        Serial.print('\t');
        Serial.print(rightEffort);
        Serial.println("");

        break;

    case NEXT:
        Serial.println("NEXT");
        leftEffort -= 10;
        rightEffort -= 10;

        Serial.print(leftEffort);
        Serial.print('\t');
        Serial.print(rightEffort);
        Serial.println("");

        break;

    default:

        if (key == NUM_0)
        {
            targetDistance = targetDistance;
            //           Serial.println(targetDistance);
            processDistanceReading(targetDistance);
        }

        if (key >= NUM_1 && key <= NUM_3)
        {

            targetDistance += 10 * (key - 15);
            //         Serial.println(targetDistance);
            processDistanceReading(targetDistance);
        }

        if (key >= NUM_4 && key <= NUM_6)
        {

            targetDistance += 10 * (key - 16);
            //        Serial.println(targetDistance);
            processDistanceReading(targetDistance);
        }

        if (key >= NUM_7 && key <= NUM_9)
        {
            targetDistance += 10 * (key - 17);
            //          Serial.println(targetDistance);
            processDistanceReading(targetDistance);
        }

        else
        {
            processDistanceReading(targetDistance);
        }
        break;
    }
}