#include "Servo.h"
#include <Wire.h>
// #include "HX711.h"

// ESCs
Servo escFront;
Servo escBack;

// Pin Mappings
const int pinRpm1 = 2;        
const int pinRpm2 = 3; 
const int pinEsc1 = 10;
const int pinEsc2 = 13;
const int pinEscCal = A3;

// I2C Addresses (Dynamically Assigned)
uint8_t addrFront = 0;
uint8_t addrBack = 0;

// RPM Tracking with Debounce
volatile unsigned long pulsePeriod1 = 0;
volatile unsigned long pulsePeriod2 = 0;
volatile unsigned long lastPulseTime1 = 0;
volatile unsigned long lastPulseTime2 = 0;
unsigned long lastSendTime = 0;

// Safety & Latching State
String inputBuffer = "";
bool systemArmed = false;
float currentOutputT1 = 0;
float currentOutputT2 = 0;

// Interrupt Service Routines for KE0068 (Active LOW on White)
void countRpm1() { 
  unsigned long now = micros();
  unsigned long interval = now - lastPulseTime1;
  if (interval > 5000) {
    pulsePeriod1 = interval; 
    lastPulseTime1 = now;
  }
}

void countRpm2() { 
  unsigned long now = micros();
  unsigned long interval = now - lastPulseTime2;
  if (interval > 5000) {
    pulsePeriod2 = interval; 
    lastPulseTime2 = now;
  }
}

// ==========================================================
// I2C Voltage Setup and Read
// ==========================================================
void initINA226(uint8_t address) {
  Wire.beginTransmission(address);
  Wire.write(0x00); // Configuration Register
  Wire.write(0x41); // MSB: 0x4127 configures continuous shunt and bus voltage
  Wire.write(0x27); // LSB
  Wire.endTransmission();
}

float readINA226Voltage(uint8_t address) {
  Wire.beginTransmission(address);
  Wire.write(0x02); // Bus Voltage Register
  
  // Return -1.0 if the device does not acknowledge the I2C address
  if (Wire.endTransmission() != 0) {
    return -1.0; 
  }
  
  Wire.requestFrom(address, (uint8_t)2);
  if (Wire.available() >= 2) {
    uint16_t val = (Wire.read() << 8) | Wire.read();
    return val * 0.00125; // 1.25mV per LSB
  }
  
  // Return -2.0 if the device acknowledged but failed to send 2 bytes
  return -2.0; 
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
      delay(50); // debounce
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
    if (millis() - t0 > 20000) {
      Serial.println("Timeout - proceeding to MIN.");
      break; 
    }
  }

  escFront.writeMicroseconds(1100);
  escBack.writeMicroseconds(1100);
  Serial.println("Output at MIN (1100us). Wait for ready beep.");
  delay(3000);
  Serial.println("Calibration finished. Continuing to normal boot.");
}

void setup() {
  Serial.begin(115200);
  
  // --- ENABLE I2C AND DYNAMIC SCANNER ---
  Wire.begin(); 
  Wire.setWireTimeout(5000, true); 

  Serial.println("Scanning I2C bus for INA226 modules...");
  for (uint8_t i = 0x40; i <= 0x4F; i++) {
    Wire.beginTransmission(i);
    if (Wire.endTransmission() == 0) {
      if (addrFront == 0) {
        addrFront = i;
        initINA226(addrFront);
        Serial.print("-> Found INA226 (Front) at 0x"); Serial.println(addrFront, HEX);
      } else if (addrBack == 0) {
        addrBack = i;
        initINA226(addrBack);
        Serial.print("-> Found INA226 (Back) at 0x"); Serial.println(addrBack, HEX);
      }
    }
  }

  if (addrFront == 0) {
    Serial.println("-> ERROR: No INA226 found! Check A0/A1 pins and SDA/SCL wiring.");
  }

  escFront.attach(pinEsc1, 1100, 1900);
  escBack.attach(pinEsc2, 1100, 1900);
  escFront.writeMicroseconds(1100);
  escBack.writeMicroseconds(1100);

  if (calibrationRequested()) {
    runEscCalibration();
  }

  pinMode(pinRpm1, INPUT_PULLUP);
  pinMode(pinRpm2, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(pinRpm1), countRpm1, FALLING);
  attachInterrupt(digitalPinToInterrupt(pinRpm2), countRpm2, FALLING);

  Serial.println("Dynamometer Hardware Initialized - RPM + VOLTAGE ONLY");
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
      else if (inputBuffer == "UNLOCK_SINGLE" || inputBuffer == "UNLOCK_CRP") {
        systemArmed = true;
      } else if (inputBuffer == "LOCK_SINGLE" || inputBuffer == "LOCK_CRP" || inputBuffer == "STOP_ALL") {
        systemArmed = false;
        currentOutputT1 = 0.0;
        currentOutputT2 = 0.0;
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

  unsigned long now = millis();
  if (now - lastSendTime >= 100) {
    lastSendTime = now;

    noInterrupts(); 
    unsigned long p1 = pulsePeriod1;
    unsigned long p2 = pulsePeriod2;
    unsigned long t1 = lastPulseTime1;
    unsigned long t2 = lastPulseTime2;
    interrupts();

    float currentRpm1 = 0;
    float currentRpm2 = 0;
    unsigned long currentMicros = micros();

    if ((currentMicros - t1 < 2000000) && p1 > 0) {
      currentRpm1 = 60000000.0 / p1;
    }
    if ((currentMicros - t2 < 2000000) && p2 > 0) {
      currentRpm2 = 60000000.0 / p2;
    }

    // Map the active addresses instead of hardcoding 0x40 and 0x41
    float valVolt1 = (addrFront != 0) ? readINA226Voltage(addrFront) : -1.0;
    float valVolt2 = (addrBack != 0) ? readINA226Voltage(addrBack) : -1.0;

    Serial.print("0.000,0.000,");
    Serial.print(currentRpm1, 0); 
    Serial.print(",");
    Serial.print(valVolt1, 2); 
    Serial.print(",0.00,0.000,0.000,");
    Serial.print(currentRpm2, 0); 
    Serial.print(",");
    Serial.print(valVolt2, 2);
    Serial.println(",0.00");                  
  }
}