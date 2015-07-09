#include <SPI.h>
#include "Adafruit_MAX31855.h"
#include "Adafruit_BLE_UART.h"

#define DO  20
#define CS  19
#define CLK 18
#define ADAFRUITBLE_REQ  10
#define ADAFRUITBLE_RDY  9
#define ADAFRUITBLE_RST  8

Adafruit_MAX31855 thermocouple(CLK, CS, DO);
Adafruit_BLE_UART BTLEserial = Adafruit_BLE_UART(ADAFRUITBLE_REQ, ADAFRUITBLE_RDY, ADAFRUITBLE_RST);

void setup() {
  Serial.begin(9600);
  Serial.println("Waiting for stabilization");
  BTLEserial.begin();
  delay(500);
}

aci_evt_opcode_t laststatus = ACI_EVT_DISCONNECTED; 

void loop() {
  BTLEserial.pollACI();
  aci_evt_opcode_t status = BTLEserial.getState();

  laststatus = status;

  if (status == ACI_EVT_CONNECTED) {
    if (BTLEserial.available()) {
      Serial.print(BTLEserial.available());
      Serial.println(F(" bytes available form BTLE"));
      }
      while (BTLEserial.available()) {
      char c = BTLEserial.read();
      Serial.print(c);
      }
    }
  if (Serial.available()){
    Serial.setTimeout(100);
    String s = Serial.readString();

    uint8_t sendbuffer[20];
    s.getBytes(sendbuffer, 20);
    char sendbuffersize = min(20, s.length());
    
    
    }
  
  Serial.print("Int. Temp = ");
  Serial.println(thermocouple.readInternal());

  double celsius = thermocouple.readCelsius();
  double fahrenheit = (9/5) * celsius + 32;
  if (isnan(celsius)) {
    Serial.println("Well f*ck me backwards");
  }
  else {
    Serial.print(celsius);
    Serial.print("°C / "); 
    Serial.print(fahrenheit);
    Serial.println("°F");
  }
  delay(1000);
}
