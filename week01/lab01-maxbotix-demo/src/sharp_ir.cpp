#include "sharp_ir.h"

#define MB_CTRL 2       //for pinging -- not a great choice since this can hamper uploading

#define sir_WINDOW_DUR 50    //ms

// Here is the mb_ez1 object that we declared as in extern in the .h file
sharp_ir sir_ez1;

// Constructor. Nothing to see here. Move along.
sharp_ir::sharp_ir(void) {}

// Default init engages all interfaces
void sharp_ir::init(void)
{
    init(USE_ADC);
}

// Allows the user to select the interface
void sharp_ir::init(uint8_t interfaces)
{

    if(interfaces & USE_ADC) //uses the MCP3002, not an onboard ADC
    {
        //SPI to talk to the MCP3002
        SPI.begin(); //defaults to VPSI: SCK, MISO, MOSI, SS; see above
        pinMode(SS, OUTPUT); //need to set the CS to OUTPUT
    }

}

uint16_t sharp_ir::readMCP3002(bool force)
{
    uint16_t retVal = 0;
    if((millis() - lastADCread >= 50) || force)
    {
        lastADCread = millis();

        // This will command the MCP to take a reading on CH0
        // Figure 6.1 of the datasheet shows the bit arrangement
        uint16_t cmdByte = 0x7800; 

        //start the SPI session
        SPISettings spiSettings; //defaults to (clk freq = 1000000, MSBFIRST, SPI_MODE0), which is what we want
        SPI.beginTransaction(spiSettings); 

        //open communication with the MCP3002
        digitalWrite(SS, LOW); 

        //this line both sends the command to read AND retrieves the result
        //the leading bits are indeterminate and need to be stripped off
        uint16_t ADCvalue = SPI.transfer16(cmdByte) & 0x03ff;

        //end communication
        digitalWrite(SS, HIGH); 

        //close the SPI session
        SPI.endTransaction(); 

        retVal = ADCvalue;
    }

    return retVal;
}

/**
 * TODO: Write a getDistance() function for the distance method of your choice.
 * 
 * getDistance should return true whenever there is a new reading, and put the result
 * in distance, which is _passed by reference_ so that you can "return" a value
 */
bool sharp_ir::getDistance(float& distance)
{
    distance = -99;
    return false;
}