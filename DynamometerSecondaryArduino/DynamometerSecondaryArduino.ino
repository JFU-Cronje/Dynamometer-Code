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

// Latest raw readings and non-blocking tare offsets
long lastRaw[4] = {0, 0, 0, 0};
long tareOffset[4] = {0, 0, 0, 0};

// Bit 0: Thrust1 ('A'), Bit 1: Torque1 ('C'), Bit 2: Thrust2 ('E'), Bit 3: Torque2 ('G')
volatile uint8_t pendingTareMask = 0;

void setup() {
  pinMode(pinHandshake, OUTPUT);
  digitalWrite(pinHandshake, LOW);

  loadCellThrust1.begin(pinThrust1_DT, pinThrust1_SCK);
  loadCellTorque1.begin(pinTorque1_DT, pinTorque1_SCK);
  loadCellThrust2.begin(pinThrust2_DT, pinThrust2_SCK);
  loadCellTorque2.begin(pinTorque2_DT, pinTorque2_SCK);

  unsigned long t0 = millis();
  while (millis() - t0 < 1000) {
    if (loadCellThrust1.is_ready() && loadCellTorque1.is_ready() &&
        loadCellThrust2.is_ready() && loadCellTorque2.is_ready()) {
      break;
    }
  }

  // Initial boot average tare
  if (loadCellThrust1.is_ready()) { lastRaw[0] = loadCellThrust1.read_average(10); tareOffset[0] = lastRaw[0]; }
  if (loadCellTorque1.is_ready()) { lastRaw[1] = loadCellTorque1.read_average(10); tareOffset[1] = lastRaw[1]; }
  if (loadCellThrust2.is_ready()) { lastRaw[2] = loadCellThrust2.read_average(10); tareOffset[2] = lastRaw[2]; }
  if (loadCellTorque2.is_ready()) { lastRaw[3] = loadCellTorque2.read_average(10); tareOffset[3] = lastRaw[3]; }

  Wire.begin(8);
  Wire.setClock(400000);
  Wire.onRequest(requestEvent); 
  Wire.onReceive(receiveEvent);
}

void loop() {
  bool updated = false;

  if (loadCellThrust1.is_ready()) { lastRaw[0] = loadCellThrust1.read(); updated = true; }
  if (loadCellTorque1.is_ready()) { lastRaw[1] = loadCellTorque1.read(); updated = true; }
  if (loadCellThrust2.is_ready()) { lastRaw[2] = loadCellThrust2.read(); updated = true; }
  if (loadCellTorque2.is_ready()) { lastRaw[3] = loadCellTorque2.read(); updated = true; }

  // Instantaneous non-blocking tare using the latest sample
  if (pendingTareMask != 0) {
    noInterrupts();
    uint8_t mask = pendingTareMask;
    pendingTareMask = 0;
    interrupts();

    if (mask & 0x01) tareOffset[0] = lastRaw[0];
    if (mask & 0x02) tareOffset[1] = lastRaw[1];
    if (mask & 0x04) tareOffset[2] = lastRaw[2];
    if (mask & 0x08) tareOffset[3] = lastRaw[3];
    updated = true;
  }

  if (updated) {
    float t1 = (float)(lastRaw[0] - tareOffset[0]);
    float q1 = (float)(lastRaw[1] - tareOffset[1]);
    float t2 = (float)(lastRaw[2] - tareOffset[2]);
    float q2 = (float)(lastRaw[3] - tareOffset[3]);

    noInterrupts();
    megaData.val[0] = t1;
    megaData.val[1] = q1;
    megaData.val[2] = t2;
    megaData.val[3] = q2;
    interrupts();

    digitalWrite(pinHandshake, HIGH);
    delayMicroseconds(500);
    digitalWrite(pinHandshake, LOW);
  }

  delay(2);
}

void requestEvent() {
  Wire.write(megaData.bytes, 16); 
}

void receiveEvent(int howMany) {
  while (Wire.available()) {
    char cmd = Wire.read();
    if (cmd == 'A') pendingTareMask |= 0x01;
    else if (cmd == 'C') pendingTareMask |= 0x02;
    else if (cmd == 'E') pendingTareMask |= 0x04;
    else if (cmd == 'G') pendingTareMask |= 0x08;
    else if (cmd == 'Z') pendingTareMask |= 0x0F;
  }
}