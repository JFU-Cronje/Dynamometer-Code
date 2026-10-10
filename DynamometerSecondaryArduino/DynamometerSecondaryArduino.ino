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

volatile char pendingCommand = 0;

void setup() {
  pinMode(pinHandshake, OUTPUT);
  digitalWrite(pinHandshake, LOW);

  loadCellThrust1.begin(pinThrust1_DT, pinThrust1_SCK);
  loadCellTorque1.begin(pinTorque1_DT, pinTorque1_SCK);
  loadCellThrust2.begin(pinThrust2_DT, pinThrust2_SCK);
  loadCellTorque2.begin(pinTorque2_DT, pinTorque2_SCK);

  Wire.begin(8); 
  Wire.onRequest(requestEvent); 
  Wire.onReceive(receiveEvent);
}

void loop() {
  // Execute Tare requests in the main loop to avoid hanging the I2C bus
  if (pendingCommand != 0) {
    char cmd = pendingCommand;
    pendingCommand = 0;
    if (cmd == 'A') loadCellThrust1.tare(1);
    if (cmd == 'C') loadCellTorque1.tare(1);
    if (cmd == 'E') loadCellThrust2.tare(1);
    if (cmd == 'G') loadCellTorque2.tare(1);
  }

  float t1 = megaData.val[0];
  float q1 = megaData.val[1];
  float t2 = megaData.val[2];
  float q2 = megaData.val[3];

  if (loadCellThrust1.is_ready()) t1 = loadCellThrust1.get_units(1);
  if (loadCellTorque1.is_ready()) q1 = loadCellTorque1.get_units(1);
  if (loadCellThrust2.is_ready()) t2 = loadCellThrust2.get_units(1);
  if (loadCellTorque2.is_ready()) q2 = loadCellTorque2.get_units(1);

  // Atomic update: disables interrupts for < 1 microsecond to prevent torn floats
  noInterrupts();
  megaData.val[0] = t1;
  megaData.val[1] = q1;
  megaData.val[2] = t2;
  megaData.val[3] = q2;
  interrupts();

  // Pulse handshake pin to inform Uno that data is updated
  digitalWrite(pinHandshake, HIGH);
  delayMicroseconds(500);
  digitalWrite(pinHandshake, LOW);
  delay(15);
}

void requestEvent() {
  Wire.write(megaData.bytes, 16); 
}

void receiveEvent(int howMany) {
  while (Wire.available()) {
    pendingCommand = Wire.read();
  }
}