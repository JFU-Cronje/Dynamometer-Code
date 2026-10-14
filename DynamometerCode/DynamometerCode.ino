#include "Servo.h"
#include <Wire.h>
#include <string.h>
#include <stdlib.h>

union FloatConverter {
  float val[4] = {0.0, 0.0, 0.0, 0.0};
  byte bytes[16];
} loadCellData;

Servo escFront;
Servo escBack;

// Pin Mappings
const int pinHandshake = 2; 
const int pinRpm2 = 3; 
const int pinRpm1 = 4;
const int pinEsc1 = 10;
const int pinEsc2 = 13;
const int pinEscCal = A3;
const int pinCurr1 = A1; 
const int pinCurr2 = A2;

const uint8_t addrVolt1 = 0x44;
const uint8_t addrVolt2 = 0x45;

// RPM Tracking with Debounce
volatile unsigned long pulsePeriod1 = 0;
volatile unsigned long pulsePeriod2 = 0;
volatile unsigned long lastPulseTime1 = 0;
volatile unsigned long lastPulseTime2 = 0;
unsigned long lastSendTime = 0;

volatile bool megaDataReady = false;

void megaReadyISR() {
  megaDataReady = true;
}

// Fixed-size serial command buffer (eliminates heap fragmentation)
char cmdBuffer[32];
uint8_t cmdIndex = 0;

bool systemArmed = false;
float currentOutputT1 = 0.0;
float currentOutputT2 = 0.0;

float scaleT1 = 1.0;
float scaleQ1 = 1.0;
float scaleT2 = 1.0;
float scaleQ2 = 1.0;

float filteredRpm1 = 0.0;
float filteredRpm2 = 0.0;

void countRpm1() { 
  unsigned long now = micros();
  unsigned long interval = now - lastPulseTime1;
  if (interval > 6000) {
    pulsePeriod1 = interval; 
    lastPulseTime1 = now;
  }
}

void countRpm2() { 
  unsigned long now = micros();
  unsigned long interval = now - lastPulseTime2;
  if (interval > 6000) {
    pulsePeriod2 = interval; 
    lastPulseTime2 = now;
  }
}

void initINA226(uint8_t address) {
  Wire.beginTransmission(address);
  Wire.write(0x00); 
  Wire.write(0x47); 
  Wire.write(0x27); 
  Wire.endTransmission();
}

float readINA226Voltage(uint8_t address) {
  Wire.beginTransmission(address);
  Wire.write(0x02); 
  if (Wire.endTransmission() != 0) return 0.0; 
  
  Wire.requestFrom(address, (uint8_t)2);
  if (Wire.available() >= 2) {
    uint16_t val = (Wire.read() << 8) | Wire.read();
    float calibratedVoltage = (val * 0.00125f) * 0.95098f; 
    return calibratedVoltage < 0.0f ? 0.0f : calibratedVoltage;
  }
  return 0.0; 
}

float readACS724Current(int pin) {
  long sum = 0;
  for (uint8_t i = 0; i < 8; i++) {
    sum += analogRead(pin);
  }
  float voltage = (sum * 0.125f) * (5.0f / 1023.0f);
  float current = (voltage - 0.5f) / 0.200f;
  return current < 0.0f ? 0.0f : current;
}

bool calibrationRequested() {
  pinMode(pinEscCal, INPUT_PULLUP);
  Serial.println("Ground Pin A3 within 3s to calibrate ESC...");
  uint32_t t0 = millis();
  while (millis() - t0 < 3000) {
    if (digitalRead(pinEscCal) == LOW) {
      delay(50); 
      if (digitalRead(pinEscCal) == LOW) return true;
    }
  }
  return false;
}

void runEscCalibration() {
  Serial.println("=== ESC CALIBRATION MODE ===");
  escFront.writeMicroseconds(1900);
  escBack.writeMicroseconds(1900);
  Serial.println("Output at MAX (1900us). Connect battery, wait for '123' then 'beep-beep'.");
  Serial.println("Remove ground from Pin A3 to proceed to MIN.");
  
  uint32_t t0 = millis();
  while (digitalRead(pinEscCal) == LOW) {
    if (millis() - t0 > 20000) break; 
  }

  escFront.writeMicroseconds(1100);
  escBack.writeMicroseconds(1100);
  Serial.println("Output at MIN (1100us). Wait for ready beep.");
  delay(3000);
  Serial.println("Calibration finished. Continuing to normal boot.");
}

void runI2CScan() {
  Serial.println("--- I2C SCAN START ---");
  byte count = 0;
  for (byte i = 1; i < 127; i++) {
    Wire.beginTransmission(i);
    if (Wire.endTransmission() == 0) {
      Serial.print("Device found at 0x");
      if (i < 16) Serial.print("0");
      Serial.println(i, HEX);
      count++;
    }
  }
  Serial.print("Total devices: ");
  Serial.println(count);
  Serial.println("--- I2C SCAN END ---");
}

void processSerialCommand(const char* cmd) {
  if (strncmp(cmd, "T1_", 3) == 0) {
    float pct = atof(cmd + 3);
    if (pct < 0.0f) pct = 0.0f;
    if (pct > 100.0f) pct = 100.0f;
    currentOutputT1 = pct;
  }
  else if (strncmp(cmd, "T2_", 3) == 0) {
    float pct = atof(cmd + 3);
    if (pct < 0.0f) pct = 0.0f;
    if (pct > 100.0f) pct = 100.0f;
    currentOutputT2 = pct;
  }
  else if (strncmp(cmd, "S_T1_", 5) == 0) scaleT1 = atof(cmd + 5);
  else if (strncmp(cmd, "S_Q1_", 5) == 0) scaleQ1 = atof(cmd + 5);
  else if (strncmp(cmd, "S_T2_", 5) == 0) scaleT2 = atof(cmd + 5);
  else if (strncmp(cmd, "S_Q2_", 5) == 0) scaleQ2 = atof(cmd + 5);
  else if (cmd[0] != '\0' && cmd[1] == '\0' &&
           (cmd[0] == 'A' || cmd[0] == 'C' || cmd[0] == 'E' || cmd[0] == 'G' || cmd[0] == 'Z')) {
    Wire.beginTransmission(8);
    Wire.write(cmd[0]);
    Wire.endTransmission();
  }
  else if (strcmp(cmd, "UNLOCK_SINGLE") == 0 || strcmp(cmd, "UNLOCK_CRP") == 0) {
    systemArmed = true;
  }
  else if (strcmp(cmd, "LOCK_SINGLE") == 0 || strcmp(cmd, "LOCK_CRP") == 0 || strcmp(cmd, "STOP_ALL") == 0) {
    systemArmed = false;
    currentOutputT1 = 0.0f;
    currentOutputT2 = 0.0f;
  }
  else if (strcmp(cmd, "SCAN_I2C") == 0) {
    runI2CScan();
  }
}

void setup() {
  Serial.begin(115200);

  Wire.begin();
  Wire.setClock(400000);
  Wire.setWireTimeout(25000, true); 
  runI2CScan();

  initINA226(addrVolt1); 
  initINA226(addrVolt2); 

  escFront.attach(pinEsc1, 1100, 1900);
  escBack.attach(pinEsc2, 1100, 1900);
  escFront.writeMicroseconds(1100);
  escBack.writeMicroseconds(1100);

  if (calibrationRequested()) {
    runEscCalibration();
  }

  pinMode(pinRpm1, INPUT_PULLUP);
  pinMode(pinRpm2, INPUT_PULLUP);
  pinMode(pinHandshake, INPUT); 
  
  attachInterrupt(digitalPinToInterrupt(pinHandshake), megaReadyISR, RISING);
  attachInterrupt(digitalPinToInterrupt(pinRpm2), countRpm2, FALLING);
  
  PCICR |= B00000100;  
  PCMSK2 |= B00010000; 

  Serial.println("Dynamometer Hardware Initialized - MAIN UNO");
}

ISR(PCINT2_vect) {
  if (digitalRead(pinRpm1) == LOW) countRpm1();
}

void loop() {
  while (Serial.available() > 0) {
    char c = Serial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      cmdBuffer[cmdIndex] = '\0';
      if (cmdIndex > 0) processSerialCommand(cmdBuffer);
      cmdIndex = 0;
    } else if (cmdIndex < sizeof(cmdBuffer) - 1) {
      cmdBuffer[cmdIndex++] = c;
    }
  }

  if (!systemArmed) {
    escFront.writeMicroseconds(1100);
    escBack.writeMicroseconds(1100);
  } else {
    escFront.writeMicroseconds((int)(1100.0f + (currentOutputT1 * 8.0f)));
    escBack.writeMicroseconds((int)(1100.0f + (currentOutputT2 * 8.0f)));
  }

  if (megaDataReady) {
    if (Wire.requestFrom(8, 16) == 16) {
      for (uint8_t i = 0; i < 16; i++) {
        loadCellData.bytes[i] = Wire.read();
      }
      loadCellData.val[0] *= scaleT1;
      loadCellData.val[1] *= scaleQ1;
      loadCellData.val[2] *= scaleT2;
      loadCellData.val[3] *= scaleQ2;
    }
    megaDataReady = false;
  }

  unsigned long now = millis();
  if (now - lastSendTime >= 100) {
    lastSendTime = now;

    noInterrupts(); 
    unsigned long p1 = pulsePeriod1;
    unsigned long p2 = pulsePeriod2;
    unsigned long t1 = lastPulseTime1;
    unsigned long t2 = lastPulseTime2;
    interrupts();

    float rawRpm1 = 0.0f;
    float rawRpm2 = 0.0f;
    unsigned long currentMicros = micros();
    const unsigned long RPM_TIMEOUT_MICROS = 250000;

    if ((currentMicros - t1 < RPM_TIMEOUT_MICROS) && p1 > 0) {
      rawRpm1 = 60000000.0f / p1;
    } else {
      pulsePeriod1 = 0; 
    }

    if ((currentMicros - t2 < RPM_TIMEOUT_MICROS) && p2 > 0) {
      rawRpm2 = 60000000.0f / p2;
    } else {
      pulsePeriod2 = 0; 
    }

    if (rawRpm1 > 0.0f) {
      if (filteredRpm1 == 0.0f && rawRpm1 > 3500.0f) rawRpm1 = 800.0f;
      filteredRpm1 = (filteredRpm1 * 0.7f) + (rawRpm1 * 0.3f);
    } else {
      filteredRpm1 = 0.0f;
    }

    if (rawRpm2 > 0.0f) {
      if (filteredRpm2 == 0.0f && rawRpm2 > 3500.0f) rawRpm2 = 800.0f;
      filteredRpm2 = (filteredRpm2 * 0.7f) + (rawRpm2 * 0.3f);
    } else {
      filteredRpm2 = 0.0f;
    }

    float valVolt1 = readINA226Voltage(addrVolt1); 
    float valVolt2 = readINA226Voltage(addrVolt2); 
    float valCurr1 = readACS724Current(pinCurr1);
    float valCurr2 = readACS724Current(pinCurr2);

    Serial.print(loadCellData.val[0], 3); 
    Serial.print(',');
    Serial.print(loadCellData.val[1], 3); 
    Serial.print(',');
    Serial.print(filteredRpm1, 0); 
    Serial.print(',');
    Serial.print(valVolt1, 2); 
    Serial.print(',');
    Serial.print(valCurr1, 2); 
    Serial.print(',');
    Serial.print(loadCellData.val[2], 3); 
    Serial.print(',');
    Serial.print(loadCellData.val[3], 3); 
    Serial.print(',');
    Serial.print(filteredRpm2, 0); 
    Serial.print(',');
    Serial.print(valVolt2, 2); 
    Serial.print(',');
    Serial.println(valCurr2, 2);            
  }
}