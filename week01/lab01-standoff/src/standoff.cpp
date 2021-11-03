#include "standoff.h"
#include "ir_codes.h"

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

void StandoffController::processDistanceReading(float distance)
{
    float error = targetDistance - distance;
    Serial.print(targetDistance);
    Serial.print('\t');
    Serial.print(error);
    Serial.print('\t');
    float effort = piStandoffer.ComputeEffort(error);

    Serial.print(effort);
    Serial.print('\t');

    leftEffort = effort;
    rightEffort = effort;
}

void StandoffController::handleKeyPress(int16_t key)
{
    switch (key)
    {

    case STOP:
        processDistanceReading(targetDistance);
        break;

    case PREV:
        targetDistance += 10;
        Serial.println(targetDistance);
        processDistanceReading(50);
        break;

    case NEXT:
        targetDistance -= 10;
        Serial.println(targetDistance);
        processDistanceReading(50);
        break;

    default:

        if (key == NUM_0)
        {
            targetDistance = targetDistance;
            Serial.println(targetDistance);
            processDistanceReading(50);
        }

        if (key >= NUM_1 && key <= NUM_3)
        {

            targetDistance += 10 * (key - 15);
            Serial.println(targetDistance);
            processDistanceReading(50);

        }

        if (key >= NUM_4 && key <= NUM_6)
        {

            targetDistance += 10 * (key-16);
            Serial.println(targetDistance);
            processDistanceReading(50);

        }

        if (key >= NUM_7 && key <= NUM_9)
        {
            targetDistance += 10 * (key-17);
            Serial.println(targetDistance);
            processDistanceReading(50);

        }
        break;
    }
}