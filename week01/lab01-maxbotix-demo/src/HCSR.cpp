#include "HCSR.h"

#define MB_CTRL 2       //for pinging -- not a great choice since this can hamper uploading
#define PULSE_PIN 16    //for reading pulse

#define MB_WINDOW_DUR 50    //ms

// Here is the mb_ez1 object that we declared as in extern in the .h file
HCSR hc_ez1;

//ISR for reading the echoes. Or is it echos?
void ISR_MaxBotix(void)
{
    hc_ez1.MB_ISR();
}

// Constructor. Nothing to see here. Move along.
HCSR::HCSR(void) {}

// Default init engages all interfaces
void HCSR::init(void)
{
    init( USE_ECHO | USE_CTRL_PIN);
}

// Allows the user to select the interface
void HCSR::init(uint8_t interfaces)
{
    if(interfaces & USE_ECHO)
    {
        // assert ECHO pin is an input
        pinMode(PULSE_PIN, INPUT);
        attachInterrupt(PULSE_PIN, ISR_MaxBotix, CHANGE);
    }

    if(interfaces & USE_CTRL_PIN)
    {
        //control pin for commanding pings
        pinMode(MB_CTRL, OUTPUT);
    }
}

/**
 * checkPingTimer check to see if it's time to send a new ping.
 * You must select USE_CTRL_PIN in init() for this to work.
 */
uint8_t HCSR::checkPingTimer(void)
{
    //check if we're ready to ping
    if(millis() - lastPing >= pingInterval)
    {
        pulseEnd = pulseStart = 0;

        //clear out any leftover states
        state = 0;

        lastPing = millis();    //not perfectly on schedule, but safer and close enough
        lastADCread = lastPing; //this will make sure the proper interval is past before we read the ADC

        digitalWrite(MB_CTRL, HIGH); //commands a ping; leave high for the duration
        delayMicroseconds(30); //datasheet says hold HIGH for >20us; we'll use 30 to be 'safe'
        digitalWrite(MB_CTRL, LOW); //unclear if pin has to stay HIGH
    }

    return state;
}

uint16_t HCSR::checkEcho(void)
{
    uint16_t echoLength = 0;
    if(state & ECHO_RECD)
    {
        echoLength = pulseEnd - pulseStart;
        state &= ~ECHO_RECD;
    }

    return echoLength;
}

//ISR for echo pin
void HCSR::MB_ISR(void)
{
    if(digitalRead(PULSE_PIN))  //transitioned to HIGH
    {
        pulseStart = micros();
    }

    else                        //transitioned to LOW
    {
        pulseEnd = micros();
        state |= ECHO_RECD;
    } 
}

/**
 * TODO: Write a getDistance() function for the distance method of your choice.
 * 
 * getDistance should return true whenever there is a new reading, and put the result
 * in distance, which is _passed by reference_ so that you can "return" a value
 */
bool HCSR::getDistance(float& distance)
{
    uint16_t pulse = HC_ez1.checkEcho();

        if(HCSR::checkEcho()){
            distance = (pulse/58);
            return true;
        }

        else{
            distance = -99;
            return false;
        }
}