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
#include <sharp_ir.h>
#include <Filter.h>

Filter filter_1;

void setup()
{
  delay(1000);
  
  Serial.begin(115200);
  mb_ez1.init();
  filter_1.init();
  Serial.println("Velkommen til");

  
  //sir_ez1.init();
}
void loop() 
{
  /**
   * For this demo, we key everything on the ascii output, since
   * that is the last thing that the sensor prepares for output.
   * Everything else should be ready at that point.
   **/
  uint16_t adcReading = mb_ez1.readMCP3002(true);
  
  if(adcReading) 
  {

    //passing true ignores the timer and forces a reading
    //from the datasheet, if the serial output is ready, the voltage is ready
    
    float measure = filter_1.takeReading();
    float med = filter_1.getMedian();
    float avg = filter_1.getAverage();
    Serial.print(adcReading); 
    Serial.print('\t');
    Serial.print(measure); //TODO: change this line to output distance in cm
    Serial.print('\t');
    Serial.print(med);
    Serial.print('\t');
    Serial.print(avg);
    Serial.print('\n');
    
  } 
  
  /**
  uint16_t adcReading = sir_ez1.readMCP3002(true);
  if(adcReading && i<250)
  {
    Serial.print(adcReading);
    Serial.print('\n');
    i++;
  } 
  **/

}
