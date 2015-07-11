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
#include <Metro.h> 



/*** Must be global ***/

float tempNow = 0;
float tempSensor = 0;
int isNegative = 0;
int readIsNegative = 0;
String s;
SnoozeBlock config;
int tempOptimal = 0;
int temptooLow = tempOptimal - 50 ;
int temptooHigh = tempOptimal + 50;
Metro serialMetro = Metro(10000);
char readBuffer[3];
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
  
  Serial1.begin(9600);
  //while(!Serial); // Leonardo/Micro should wait for serial init
  Serial1.println(F("Adafruit Bluefruit Low Energy nRF8001 Print echo demo"));

  BTLEserial.setDeviceName("BBQTemp"); /* 7 characters max! */

  pinMode(3,INPUT);
  config.pinMode(3,INPUT_PULLUP,RISING);
  BTLEserial.begin();
  //config.setTimer(5000);

  // //Get the radio advertising before we sleep
  while (status != ACI_EVT_DEVICE_STARTED) {
   status = BTLEserial.getState();
   BTLEserial.pollACI();
   
  }
  Serial1.println("BLE started");
  tempOptimal = analogRead(A0) * (3.3 / 1023.0);
  tempOptimal = ((tempOptimal - 1.25) / 0.005 ) ;
  
}

/**************************************************************************/
/*!
    Constantly checks for new events on the nRF8001
    */
    /**************************************************************************/
aci_evt_opcode_t laststatus = ACI_EVT_DISCONNECTED;

void loop()
{ 

  //int tempJustRight = 0; 
  isNegative = 0;

  // Tell the nRF8001 to do whatever it should be working on.
  BTLEserial.pollACI();

  // Ask what is our current status
  status = BTLEserial.getState();
  // If the status changed....
  if (status != laststatus) {
    // print it out!s
    if (status == ACI_EVT_DEVICE_STARTED) {
      Serial1.println(F("* Advertising started"));
      if (digitalRead(3) == HIGH) {
      Serial1.println("SLEEP_started");
      Serial1.flush();
      Snooze.sleep(config);
      Serial1.println("WAKE_started");
      }
  
      //Serial.println(F("*sleeping"));
      //Snooze.sleep(config);
      //Serial.println(F("awake"));

    }
    if (status == ACI_EVT_CONNECTED) {
      Serial1.println(F("* Connected!"));
    }
    if (status == ACI_EVT_DISCONNECTED) {
      Serial1.println(F("* Disconnected or advertising timed out"));
      
     // if (digitalRead(3) == HIGH) {
      // Serial1.println("SLEEP_Disconnected");
      // Serial1.flush();
      // Snooze.sleep(config);
      // Serial1.println("WAKE_disconnected");
    //}
    }
    // OK set the last status change to this one
    laststatus = status;
  }

  if (status == ACI_EVT_CONNECTED) {
    // Lets see if there's any data for us!
    // OK while we still have something to read, get a character and print it out
    int i = 0;
    while (BTLEserial.available()) {
      char c = BTLEserial.read();
      if(c = '-'){
         readIsNegative = 1;     
      }
      else{
        readBuffer[i] = c;
        i++;
      }
    }

    if(readIsNegative){
      tempOptimal = -1 * ( (readBuffer[0] - '0') * 100 + ( readBuffer[1] - '0') * 10 + ( readBuffer[2] - '0') );
    }
    else{
      tempOptimal = ( (readBuffer[0] - '0') * 100 + ( readBuffer[1] - '0') * 10 + ( readBuffer[2] - '0') );
    }

    //Serial.print(tempOptimal);
    if (serialMetro.check() == 1) { 
      tempSensor = analogRead(A0);
      tempSensor = analogRead(A0) * (3.3 / 1023.0);
      tempNow = ( (tempSensor - 1.25) / 0.005 ) ;
      if(tempNow < 0){
        isNegative = 1;
        tempNow = tempNow * - 1.0;
      }

      char buffer[5];
      unsigned char sendbuffer[10];
      s = dtostrf(tempNow,5,2,buffer);
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
