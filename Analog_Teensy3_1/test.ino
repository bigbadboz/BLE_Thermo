#include <SPI.h>
#include "Adafruit_BLE_UART.h"
#include <Snooze.h>

// nRF8001 pins
#define NRF_RQN 10
#define NRF_RDY 2
#define NRF_RST 9


Adafruit_BLE_UART BTLEserial = Adafruit_BLE_UART(NRF_RQN, NRF_RDY, NRF_RST);
aci_evt_opcode_t status;
SnoozeBlock config;

void setup(void)
{ 
  Serial3.begin(38400);
  delay(1000);
  Serial3.println(F("Adafruit Bluefruit Low Energy nRF8001 Print echo demo"));
  
  pinMode(NRF_RDY, INPUT_PULLUP);
  config.pinMode(NRF_RDY, INPUT_PULLUP, LOW);

  BTLEserial.begin();
  
  // Get the radio advertising before we sleep
  while (status != ACI_EVT_DEVICE_STARTED) {
    status = BTLEserial.getState();
    BTLEserial.pollACI();
  }
}

aci_evt_opcode_t laststatus = ACI_EVT_DISCONNECTED;

void loop()
{
  if (digitalRead(NRF_RDY) == HIGH) {
    Serial3.println("SLEEP");
    Serial3.flush();
    Snooze.sleep(config);
    Serial3.println("WAKE");
  }
  // Tell the nRF8001 to do whatever it should be working on.
  BTLEserial.pollACI();

  // Ask what is our current status
  status = BTLEserial.getState();
  // If the status changed....
  if (status != laststatus) {
    // print it out!
    if (status == ACI_EVT_DEVICE_STARTED) {
        Serial3.println(F("* Advertising started"));
    }
    if (status == ACI_EVT_CONNECTED) {
        Serial3.println(F("* Connected!"));
    }
    if (status == ACI_EVT_DISCONNECTED) {
        Serial3.println(F("* Disconnected or advertising timed out"));
    }
    // OK set the last status change to this one
    laststatus = status;
  }

  if (status == ACI_EVT_CONNECTED) {
    // Lets see if there's any data for us!
    if (BTLEserial.available()) {
      Serial3.print("* "); Serial3.print(BTLEserial.available()); Serial3.println(F(" bytes available from BTLE"));
    }
    // OK while we still have something to read, get a character and print it out
    while (BTLEserial.available()) {
      char c = BTLEserial.read();
      Serial3.print(c);
    }

    // Next up, see if we have any data to get from the Serial console

    if (Serial3.available()) {
      // Read a line from Serial
      Serial3.setTimeout(100); // 100 millisecond timeout
      String s = Serial3.readString();

      // We need to convert the line to bytes, no more than 20 at this time
      uint8_t sendbuffer[20];
      s.getBytes(sendbuffer, 20);
      char sendbuffersize = min(20, s.length());

      Serial3.print(F("\n* Sending -> \"")); Serial3.print((char *)sendbuffer); Serial3.println("\"");

      // write the data
      BTLEserial.write(sendbuffer, sendbuffersize);
    }
  }
}