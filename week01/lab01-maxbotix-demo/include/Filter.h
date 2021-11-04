#pragma once

#include <Arduino.h>
#include <MaxBotix.h>

class Filter
{
    private:
    float lastMed = 0;
    float lastAvg = 0;
    float last[5];
    char count = 0;
    public:
    
    Filter(void);
    void init(void);
    float getMedian(void);
    float getAverage(void);
    float takeReading(void);
};
