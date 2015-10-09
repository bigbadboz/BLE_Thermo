#include <SPI.h>
#include "Adafruit_MAX31855.h"
#include "Adafruit_BLE_UART.h"
#include <Metro.h> 
#include <ADC.h>
#define DO  23
#define CS  22
#define CLK 21
#define ADAFRUITBLE_REQ  10
#define ADAFRUITBLE_RDY  2
#define ADAFRUITBLE_RST  9

Adafruit_MAX31855 thermocouple(CLK, CS, DO);
Adafruit_BLE_UART BTLEserial = Adafruit_BLE_UART(ADAFRUITBLE_REQ, ADAFRUITBLE_RDY, ADAFRUITBLE_RST);

double digitalTemp = 0;
float analogTemp = 0;
int isAnalogNegative = 0;
int isDigitalNegative=0;
String d; //digital temp
String a; //analog temp

Metro serialMetro = Metro(1000);
ADC *adc = new ADC(); // adc object;
aci_evt_opcode_t status;


void setup() {
  Serial.begin(9600);
  while(!Serial);
  Serial.println("Waiting for stabilization");
  BTLEserial.setDeviceName("BBQBOZ"); 
  BTLEserial.begin();
}

aci_evt_opcode_t laststatus = ACI_EVT_DISCONNECTED; 

void loop() {
  BTLEserial.pollACI();
  status = BTLEserial.getState();
  
 

  if (status == ACI_EVT_CONNECTED) {
        int i = 0;
    if (serialMetro.check() == 1) { 
      
      isAnalogNegative = 0;
      analogTemp = ( ( adc->analogRead(A0)  * ( 3.3/adc->getMaxValue(ADC_0) ) ) - 1.25) / 0.005  ;
      if(analogTemp < 0.0){
        isAnalogNegative = 1;
        analogTemp = abs(analogTemp);
      }
      char analogBuffer[5];
      unsigned char analogSendBuffer[5];
      a = dtostrf(analogTemp,4,1,analogBuffer);
      if(isAnalogNegative){
        a = "-" + a;
      }
      else{
        a = "A" + a;
      }
      a.getBytes(analogSendBuffer, 5);
      char analogSendBufferSize = min(5, a.length());
      BTLEserial.write(analogSendBuffer, analogSendBufferSize);

      isDigitalNegative = 0;
      digitalTemp = thermocouple.readCelsius();
      if(digitalTemp < 0.0){
        isDigitalNegative = 1;
        digitalTemp = abs(digitalTemp);
      }
      char digitalBuffer[5];
      unsigned char digitalSendBuffer[5];
      d = dtostrf(digitalTemp,4,1,digitalBuffer);
      Serial.println(a);
      
      if(isDigitalNegative){
        d = "-" + d;
      }
      else{
        d = "D" + d;
      }
       d.getBytes(digitalSendBuffer, 5);
      char digitalSendBufferSize = min(5, d.length());
      BTLEserial.write(digitalSendBuffer, digitalSendBufferSize);
      Serial.println(a);
      Serial.println(d);
    }
  }

}