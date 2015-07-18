#include <SPI.h>
#include <Adafruit_BLE_UART.h>
#include <Snooze.h>
#include <Metro.h> 
#include <ADC.h>

float tempNow = 0;
float tempSensor = 0;
int isNegative = 0;
int readIsNegative = 0;
String s;
SnoozeBlock config;
Metro serialMetro = Metro(1000);
ADC *adc = new ADC(); // adc object;

// Connect CLK/MISO/MOSI to hardware SPI
// e.g. On UNO & compatible: CLK = 13, MISO = 12, MOSI = 11
#define ADAFRUITBLE_REQ 10
#define ADAFRUITBLE_RDY 2     // This should be an interrupt pin, on Uno thats #2 or #3
#define ADAFRUITBLE_RST 9

Adafruit_BLE_UART BTLEserial = Adafruit_BLE_UART(ADAFRUITBLE_REQ, ADAFRUITBLE_RDY, ADAFRUITBLE_RST);
aci_evt_opcode_t status;

/**************************************************************************/
/*!
    Configure the Arduino and start advertising with the radio
    */
    /**************************************************************************/
void setup(void)
{ 
  //Serial.println(F("Adafruit Bluefruit Low Energy nRF8001 Print echo demo"));
  pinMode(A0, INPUT); //pin 23 single ended
  adc->setAveraging(4); // set number of averages
  adc->setResolution(10); // set bits of resolution
  adc->setConversionSpeed(ADC_LOW_SPEED); // change the conversion speed
  BTLEserial.setDeviceName("BBQHAN"); /* 7 characters max! */

  BTLEserial.begin();  
}

/**************************************************************************/
/*!
    Constantly checks for new events on the nRF8001
    */
    /**************************************************************************/
aci_evt_opcode_t laststatus = ACI_EVT_DISCONNECTED;

void loop()
{ 
  //Serial.println( ( adc->analogRead(A0) ) *3.3/adc->getMaxValue(ADC_0), DEC);
  isNegative = 0;

  // Tell the nRF8001 to do whatever it should be working on.
  BTLEserial.pollACI();

  // Ask what is our current status
  status = BTLEserial.getState();
  // If the status changed....
  if (status != laststatus) {
    laststatus = status;
  }

  if (status == ACI_EVT_CONNECTED) {
    int i = 0;
    if (serialMetro.check() == 1) { 
      isNegative = 0;
      tempSensor = adc->analogRead(A0)  * (3.3/adc->getMaxValue(ADC_0));
      tempNow = ( (tempSensor - 1.25) / 0.005 ) - 6;
      if(tempNow < 0.0){
        isNegative = 1;
        tempNow = abs(tempNow);
      } 

      char buffer[5];
      unsigned char sendbuffer[5];
      s = dtostrf(tempNow,4,1,buffer);
      if(isNegative){
        s = "-" + s;
      }
      s.getBytes(sendbuffer, 5);
      char sendbuffersize = min(5, s.length());
      // write the data
    
      BTLEserial.write(sendbuffer, sendbuffersize);
    }
  }
}
