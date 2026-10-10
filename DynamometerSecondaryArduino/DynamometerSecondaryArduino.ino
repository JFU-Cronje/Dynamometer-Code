#include <Wire.h>
#include "HX711.h"

const int pinThrust1_DT = 2;
const int pinThrust1_SCK = 3;
const int pinTorque1_DT = 4;
const int pinTorque1_SCK = 5;
const int pinThrust2_DT = 6;
const int pinThrust2_SCK = 7;
const int pinTorque2_DT = 8;
const int pinTorque2_SCK = 9;

const int pinHandshake = 12;

HX711 loadCellThrust1;
HX711 loadCellTorque1;
HX711 loadCellThrust2;
HX711 loadCellTorque2;

union FloatConverter {
  float val[4] = {0.0, 0.0, 0.0, 0.0};
  byte bytes[16];
} megaData;

void setup() {
  pinMode(pinHandshake, OUTPUT);
  digitalWrite(pinHandshake, LOW);

  loadCellThrust1.begin(pinThrust1_DT, pinThrust1_SCK);
  loadCellTorque1.begin(pinTorque1_DT, pinTorque1_SCK);
  loadCellThrust2.begin(pinThrust2_DT, pinThrust2_SCK);
  loadCellTorque2.begin(pinTorque2_DT, pinTorque2_SCK);

  Wire.begin(8); 
  Wire.onRequest(requestEvent); 
  Wire.onReceive(receiveEvent); // Listen for Tare commands from Uno
}

void loop() {
  if (loadCellThrust1.is_ready()) megaData.val[0] = loadCellThrust1.get_units(1);
  if (loadCellTorque1.is_ready()) megaData.val[1] = loadCellTorque1.get_units(1);
  if (loadCellThrust2.is_ready()) megaData.val[2] = loadCellThrust2.get_units(1);
  if (loadCellTorque2.is_ready()) megaData.val[3] = loadCellTorque2.get_units(1);

  // Signal Uno that it is safe to request I2C data
  digitalWrite(pinHandshake, HIGH);
  
  unsigned long startWait = millis();
  while(millis() - startWait < 15) { 
    delay(1); 
  }

  digitalWrite(pinHandshake, LOW);
  delay(5);
}

void requestEvent() {
  Wire.write(megaData.bytes, 16); 
}

void receiveEvent(int howMany) {
  while (Wire.available()) {
    char c = Wire.read();
    if (c == 'A') loadCellThrust1.tare();
    if (c == 'C') loadCellTorque1.tare();
    if (c == 'E') loadCellThrust2.tare();
    if (c == 'G') loadCellTorque2.tare();
  }
}