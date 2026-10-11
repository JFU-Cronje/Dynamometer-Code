#include "Servo.h"
#include <Wire.h>

// Initialize to 0.0 to prevent NaN outputs before the first read
union FloatConverter {
  float val[4] = {0.0, 0.0, 0.0, 0.0};
  byte bytes[16];
} loadCellData;

// ESCs
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

const uint8_t addrVolt1 = 0x44; // Front Voltage Sensor (A1 -> VCC, A0 -> GND)
const uint8_t addrVolt2 = 0x45; // Back Voltage Sensor  (A1 -> VCC, A0 -> VCC)

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

// Safety & Latching State
String inputBuffer = "";
bool systemArmed = false;
float currentOutputT1 = 0;
float currentOutputT2 = 0;

float scaleT1 = 1.0;
float scaleQ1 = 1.0;
float scaleT2 = 1.0;
float scaleQ2 = 1.0;

bool lastMegaState = LOW;

float filteredRpm1 = 0.0;
float filteredRpm2 = 0.0;

// Interrupt Service Routines
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

// ==========================================================
// I2C Voltage Setup and Read
// ==========================================================
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
  if (Wire.endTransmission() != 0) return -1.0; 
  
  delayMicroseconds(200);
  Wire.requestFrom(address, (uint8_t)2);
  if (Wire.available() >= 2) {
    uint16_t val = (Wire.read() << 8) | Wire.read();
    float rawVoltage = val * 0.00125; 
    float calibratedVoltage = rawVoltage * 0.95098; 
    if (calibratedVoltage < 0.0) return 0.0;
    return calibratedVoltage;
  }
  return -2.0; 
}

// ==========================================================
// Analog Current Read
// ==========================================================
float readACS724Current(int pin) {
  long sum = 0;
  for (int i = 0; i < 10; i++) {
    sum += analogRead(pin);
    delayMicroseconds(200); 
  }
  float voltage = (sum / 10.0) * (5.0 / 1023.0);
  float current = (voltage - 0.5) / 0.200;
  // Standard noise filter (reverted from 0.45A)
  if (current < 0.10 && current > -0.10) return 0.0;
  return current;
}

// ==========================================================
// ESC Throttle-Range Calibration
// ==========================================================
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

void setup() {
  Serial.begin(115200);

  Wire.begin(); 
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
    if (c == '\n') {
      inputBuffer.trim();
      
      if (inputBuffer.startsWith("T1_")) {
        float throttlePct = inputBuffer.substring(3).toFloat();
        if (throttlePct < 0.0) throttlePct = 0.0;
        if (throttlePct > 100.0) throttlePct = 100.0;
        currentOutputT1 = throttlePct;
      } else if (inputBuffer.startsWith("T2_")) {
        float throttlePct = inputBuffer.substring(3).toFloat();
        if (throttlePct < 0.0) throttlePct = 0.0;
        if (throttlePct > 100.0) throttlePct = 100.0;
        currentOutputT2 = throttlePct;
      } 
      else if (inputBuffer.startsWith("S_T1_")) scaleT1 = inputBuffer.substring(5).toFloat();
      else if (inputBuffer.startsWith("S_Q1_")) scaleQ1 = inputBuffer.substring(5).toFloat();
      else if (inputBuffer.startsWith("S_T2_")) scaleT2 = inputBuffer.substring(5).toFloat();
      else if (inputBuffer.startsWith("S_Q2_")) scaleQ2 = inputBuffer.substring(5).toFloat();
      
      else if (inputBuffer == "A" || inputBuffer == "C" || inputBuffer == "E" || inputBuffer == "G") {
        Wire.beginTransmission(8);
        Wire.write(inputBuffer.charAt(0));
        Wire.endTransmission();
      }

      else if (inputBuffer == "UNLOCK_SINGLE" || inputBuffer == "UNLOCK_CRP") systemArmed = true;
      else if (inputBuffer == "LOCK_SINGLE" || inputBuffer == "LOCK_CRP" || inputBuffer == "STOP_ALL") {
        systemArmed = false;
        currentOutputT1 = 0.0;
        currentOutputT2 = 0.0;
      }
      else if (inputBuffer == "SCAN_I2C") {
        runI2CScan();
      }  
      inputBuffer = "";
    } else {
      inputBuffer += c; 
    }
  }

  if (!systemArmed) {
    escFront.writeMicroseconds(1100);
    escBack.writeMicroseconds(1100);
  } else {
    escFront.writeMicroseconds(1100 + (currentOutputT1 * 8.0));
    escBack.writeMicroseconds(1100 + (currentOutputT2 * 8.0));
  }

  if (megaDataReady) {
    if (Wire.requestFrom(8, 16) == 16) {
      for (int i = 0; i < 16; i++) {
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

    float rawRpm1 = 0.0;
    float rawRpm2 = 0.0;
    unsigned long currentMicros = micros();

    const unsigned long RPM_TIMEOUT_MICROS = 250000;

    if ((currentMicros - t1 < RPM_TIMEOUT_MICROS) && p1 > 0) {
      rawRpm1 = 60000000.0 / p1;
    } else {
      rawRpm1 = 0.0;
      pulsePeriod1 = 0; 
    }

    if ((currentMicros - t2 < RPM_TIMEOUT_MICROS) && p2 > 0) {
      rawRpm2 = 60000000.0 / p2;
    } else {
      rawRpm2 = 0.0;
      pulsePeriod2 = 0; 
    }

    if (rawRpm1 > 0) filteredRpm1 = (filteredRpm1 * 0.7) + (rawRpm1 * 0.3);
    else filteredRpm1 = 0;

    if (rawRpm2 > 0) filteredRpm2 = (filteredRpm2 * 0.7) + (rawRpm2 * 0.3);
    else filteredRpm2 = 0;

    float valVolt1 = readINA226Voltage(addrVolt1); 
    float valVolt2 = readINA226Voltage(addrVolt2); 
    float valCurr1 = readACS724Current(pinCurr1);
    float valCurr2 = readACS724Current(pinCurr2);

    // Format: Thrust1, Torque1, RPM1, Volt1, Curr1, Thrust2, Torque2, RPM2, Volt2, Curr2
    Serial.print(loadCellData.val[0], 3); 
    Serial.print(",");
    Serial.print(loadCellData.val[1], 3); 
    Serial.print(",");
    Serial.print(filteredRpm1, 0); 
    Serial.print(",");
    Serial.print(valVolt1, 2); 
    Serial.print(",");
    Serial.print(valCurr1, 2); 
    Serial.print(",");
    Serial.print(loadCellData.val[2], 3); 
    Serial.print(",");
    Serial.print(loadCellData.val[3], 3); 
    Serial.print(",");
    Serial.print(filteredRpm2, 0); 
    Serial.print(",");
    Serial.print(valVolt2, 2); 
    Serial.print(",");
    Serial.println(valCurr2, 2);            
  }
}