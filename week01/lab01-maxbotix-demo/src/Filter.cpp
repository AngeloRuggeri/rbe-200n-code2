#include "Filter.h"

Filter::Filter(void)
{}

void Filter::init(void)
{
    char n;
    for (n = 0; n < 5; n++)
    last[n] = 0;
}
float Filter::getMedian(void)
{
    float Median = 0;
    if (last[4] != 0)
    {

        char i;
        char j;
        int temp = 0;
        if (lastMed == 0)
        {
            for (i = 0; i < 5; i++)
            {
                for (j = i + 1; j < 5; j++)
                {
                    if (last[i] > last[j])
                    {
                        temp = last[i];
                        last[i] = last[j];
                        last[j] = temp;
                    }
                }
            }
            Median = last[2];
            lastMed = Median;
            return Median;
        }
        else
        {
            char n;
            float arr[6];
            for (n = 0; n < 5; n++)
            {
                arr[n] = last[n];
            }
            arr[5] = lastMed;
            for (i = 0; i < 6; i++)
            {
                for (j = i + 1; j < 6; j++)
                {
                    if (arr[i] > arr[j])
                    {
                        temp = arr[i];
                        arr[i] = arr[j];
                        arr[j] = temp;
                    }
                }
            }
            Median = (arr[2] + arr[3]) / 2;
            lastMed = Median;
            return Median;
        }
    }
    else
    {
        Median = last[0];
        return Median;
    }
}

float Filter::getAverage(void)
{
    float Average = 0;
    float sum = 0;
    char n;
    if (lastAvg == 0)
    {
        for (n = 0; n < 5; n++)
        {
            sum = last[n] + sum;
        }
        Average = sum / 5;
        lastAvg = Average;
        return Average;
    }
    else
    {
        char n;
        float arr[6];
        for (n = 0; n < 5; n++)
        {
            arr[n] = last[n];
        }
        arr[5] = lastAvg;
        for (n = 0; n < 6; n++)
        {
            sum = arr[n] + sum;
        }
        Average = sum / 6;
        lastAvg = Average;
        return Average;
    }
}

float Filter::takeReading(void)
{
    float distance;
    if (mb_ez1.getDistance(distance))
    {
        last[count] = distance;
        count++;
        if (count == 5)
        {
            count = 0;
        }
        return distance;
    }
    else
    {
        return 0.00;
    }
}
