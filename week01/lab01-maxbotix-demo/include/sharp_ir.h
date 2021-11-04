#ifndef __SHARPIR_H
#define __SHARPIR_H

#include <Arduino.h>
#include <SPI.h>

#define ADC_READ    0x08

#define USE_ADC         0x08

class sharp_ir 
{
private:
    uint8_t state = 0;

    uint32_t lastPing = 0;          // for keeping track of intervals
    uint32_t pingInterval = 200;    // default to 200 ms

    uint32_t lastADCread = 0;       // can't read the ADC too fast

    uint32_t pulseStart = 0;
    uint32_t pulseEnd = 0;

public:
    sharp_ir(void);  //ideally, this would take pins as parameters, but just hard-coded for now since we only have one
    void init(void);
    void init(uint8_t interfaces);

    //Reads the MCP3002 ADC; returns ADC result
    uint16_t readMCP3002(bool force = false);

    /**
     * TODO: Write a getDistance() function for the distance method of your choice.
     * 
     * See the .cpp file.
     */
    bool getDistance(float& distance);

    //ISR for the MaxBotix sensor
    void MB_ISR(void);
};

extern sharp_ir sir_ez1; 

#endif