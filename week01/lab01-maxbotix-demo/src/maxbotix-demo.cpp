/*
 * Code for interfacing with the MCP3002 ADC using the SPI bus on the ESP32.
 * This code is written to use specific pins:
 * 
 * MISO = 19
 * MOSI = 23
 * SCLK = 18
 * SS   = 5
 * 
 * RX   = 16 (inverted for MaxBotix!)
 * (best not to use TX-17 for anything else, since the uart will likely conflict with pin operation)
 * 
 * pulse width is read on: 35
 * ultrasonic control pin: 2
 */

#include <Arduino.h>
#include <SPI.h>

#include <MaxBotix.h>

void setup()
{
  delay(1000);
  Serial.begin(115200);
  Serial.println("Velkommen til"); 
  mb_ez1.init();
  
}


void loop() 
{
  static int i=0;


  /**
   * For this demo, we key everything on the ascii output, since
   * that is the last thing that the sensor prepares for output.
   * Everything else should be ready at that point.
   */
  uint16_t asciiResponse = mb_ez1.readASCII();
  

  if(asciiResponse && i<250) 
  {
    i++;
    Serial.print(asciiResponse);
    Serial.print('\t');
    Serial.print(asciiResponse*2.54); //TODO: change this line to output distance in cm
    Serial.print('\t');

    uint32_t pulseLen = mb_ez1.checkEcho();
    Serial.print(pulseLen);
    Serial.print('\t');
    Serial.print((pulseLen/147)*2.54); //TODO: change this line to output distance in cm
    Serial.print('\t');

    //passing true ignores the timer and forces a reading
    //from the datasheet, if the serial output is ready, the voltage is ready
    uint16_t adcReading = mb_ez1.readMCP3002(true);
    Serial.print(adcReading); 
    Serial.print('\t');
    Serial.print((adcReading/1024.00) * 3.3 * 1000.00 * 2.54 / (6.4)); //TODO: change this line to output distance in cm
    Serial.print('\n');

    if(i==250) {
     Serial.print(i);
     Serial.print('\n');
    }

  }
}
