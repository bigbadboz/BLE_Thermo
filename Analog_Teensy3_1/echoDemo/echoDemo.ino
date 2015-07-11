/*********************************************************************
This is an example for our nRF8001 Bluetooth Low Energy Breakout

  Pick one up today in the adafruit shop!
  ------> http://www.adafruit.com/products/1697

Adafruit invests time and resources providing this open source code, 
please support Adafruit and open-source hardware by purchasing 
products from Adafruit!

Written by Kevin Townsend/KTOWN  for Adafruit Industries.
MIT license, check LICENSE for more information
All text above, and the splash screen below must be included in any redistribution
*********************************************************************/

// This version uses the internal data queing so you can treat it like Serial (kinda)!

#include <SPI.h>
#include <Adafruit_BLE_UART.h>
#include <Snooze.h>


/*** Must be global ***/
SnoozeBlock config;
float tempPast = 0;
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
  
  Serial.begin(38400);
  while(!Serial); // Leonardo/Micro should wait for serial init
  Serial.println(F("Adafruit Bluefruit Low Energy nRF8001 Print echo demo"));

  BTLEserial.setDeviceName("BBQTemp"); /* 7 characters max! */

  //pinMode(ADAFRUITBLE_RDY, INPUT_PULLUP);
  //config.pinMode(ADAFRUITBLE_RDY,INPUT_PULLUP,RISING);
  BTLEserial.begin();
  //config.setTimer(10000);

  
  //Get the radio advertising before we sleep
  // while (status != ACI_EVT_DEVICE_STARTED) {
  //  status = BTLEserial.getState();
  //  BTLEserial.pollACI();
  //  Serial.println("BLE started");
  //}
  
 }

/**************************************************************************/
/*!
    Constantly checks for new events on the nRF8001
    */
    /**************************************************************************/
aci_evt_opcode_t laststatus = ACI_EVT_DISCONNECTED;

void loop()
{ 

  //int tempMinimum = 200;
  //int tempOptimal = 275;
  //int tempMax = 350;
  //int temptooLow = 0;
  //int temptooHigh = 0;
  //int tempJustRight = 0; 
  int isNegative = 0;

  float tempSensor = analogRead(A0) * (3.3 / 1023.0);
  float tempNow = ( (tempSensor - 1.25) / 0.005 ) ;
  if(tempNow < 0){
    isNegative = 1;
    tempNow = tempNow * - 1.0;
  }
  // if (digitalRead(ADAFRUITBLE_RDY) == HIGH) {
  //   Serial.println("SLEEP");
  //   Serial.flush();
  //   Snooze.sleep(config);
  //   Serial.println("WAKE");
  // }
  // Tell the nRF8001 to do whatever it should be working on.
  BTLEserial.pollACI();

  // Ask what is our current status
  status = BTLEserial.getState();
  // If the status changed....
  if (status != laststatus) {
    // print it out!s
    if (status == ACI_EVT_DEVICE_STARTED) {
      Serial.println(F("* Advertising started"));
    }
    if (status == ACI_EVT_CONNECTED) {
      Serial.println(F("* Connected!"));
    }
    if (status == ACI_EVT_DISCONNECTED) {
      Serial.println(F("* Disconnected or advertising timed out"));
    }
    // OK set the last status change to this one
    laststatus = status;
  }

  if (status == ACI_EVT_CONNECTED) {
    // Lets see if there's any data for us!
    if (BTLEserial.available()) {
      //Serial.print("* "); Serial.print(BTLEserial.available()); Serial.println(F(" bytes available from BTLE"));
    }
    // OK while we still have something to read, get a character and print it out
    while (BTLEserial.available()) {
      char c = BTLEserial.read();
      Serial.print(c);
    }

    if(abs(tempNow - tempPast) > 2.0 ){
      char buffer[5];
      unsigned char sendbuffer[10];
      String s = dtostrf(tempNow,5,2,buffer);
      if(isNegative){
        s = "-" + s;
      }
      s.getBytes(sendbuffer, 5);
      char sendbuffersize = min(5, s.length());
      // write the data
      BTLEserial.write(sendbuffer, sendbuffersize);
      //Snooze.sleep(config);
      // digitalWrite(LED_BUILTIN, HIGH);
      // delay(1000);
      // digitalWrite(LED_BUILTIN, LOW);
      tempPast = tempNow;
    }

    
  }
}
