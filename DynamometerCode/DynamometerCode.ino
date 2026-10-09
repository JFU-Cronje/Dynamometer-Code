#include "Servo.h"
#include <Wire.h>
#include <SoftwareSerial.h>
SoftwareSerial megaSerial(4, 5); // RX on Pin 4, TX on Pin 5 (TX unused but required by library)

String megaBuffer = "";
float megaThrust1 = 0.000;
float megaTorque1 = 0.000;
float megaThrust2 = 0.000;
float megaTorque2 = 0.000;

// ESCs
Servo escFront;
Servo escBack;

// Pin Mappings
const int pinRpm1 = 2;        
const int pinRpm2 = 3; 
const int pinEsc1 = 10;
const int pinEsc2 = 13;
const int pinEscCal = A3;
const int pinCurr1 = A1; // Adjust to your actual analog pin
const int pinCurr2 = A2; // Adjust to your actual analog pin

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
  Wire.write(0x47); // MSB: 0x4727 configures 64x hardware averaging
  Wire.write(0x27); // LSB: 1.1ms conversion time, continuous mode
  Wire.endTransmission();
}

float readINA226Voltage(uint8_t address) {
  Wire.beginTransmission(address);
  Wire.write(0x02); // Bus Voltage Register
  
  if (Wire.endTransmission() != 0) {
    return -1.0; 
  }
  
  delay(1);
  
  Wire.requestFrom(address, (uint8_t)2);
  if (Wire.available() >= 2) {
    uint16_t val = (Wire.read() << 8) | Wire.read();
    float rawVoltage = val * 0.00125; // 1.25mV per LSB
    
    // Apply proportional scale factor instead of static offset
    // Multiplier = Actual (13.97) / Raw (14.69) = ~0.95098
    float calibratedVoltage = rawVoltage * 0.95098; 
    
    if (calibratedVoltage < 0.0) return 0.0;
    
    return calibratedVoltage;
  }
  
  return -2.0; 
}

// ==========================================================
// Analog Current Read (ACS724-20AU Unidirectional)
// ==========================================================
float readACS724Current(int pin) {
  // Take an average of 10 samples to smooth out motor noise
  long sum = 0;
  for (int i = 0; i < 10; i++) {
    sum += analogRead(pin);
  }
  float avgRaw = sum / 10.0;
  
  // Convert 10-bit ADC value (0-1023) to Voltage (0-5V)
  float voltage = avgRaw * (5.0 / 1023.0);
  
  // For ACS724-20AU (Unidirectional): Offset = 0.5V, Sensitivity = 0.200 V/A
  float current = (voltage - 0.5) / 0.200;
  
  // Filter out floating noise around 0 Amps
  if (current < 0.15 && current > -0.15) {
    return 0.0;
  }
  
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
  
  Wire.begin(); 
  Wire.setWireTimeout(25000, true); 

  // Run a definitive I2C scan at boot to confirm the physical address
  Serial.println("--- I2C SCAN START ---");
  for (byte i = 8; i < 120; i++) {
    Wire.beginTransmission(i);
    if (Wire.endTransmission() == 0) {
      Serial.print("Device found at 0x");
      Serial.println(i, HEX);
    }
  }
  Serial.println("--- I2C SCAN END ---");

  // Explicitly initialize the known addresses
  initINA226(0x44); // Updated from 0x40
  initINA226(0x41); // Update this to 0x45 if your second sensor is similarly configured

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

  Serial.println("Dynamometer Hardware Initialized - MAIN UNO (RPM, V, A, + MEGA LINK)");

  megaSerial.begin(38400); // Dedicated data link to the Mega
}

void loop() {
  // Parse incoming commands from PC
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

  // Parse incoming load cell data asynchronously from Mega
  while (megaSerial.available() > 0) {
    char c = megaSerial.read();
    if (c == '\n') {
      megaBuffer.trim();
      int idx1 = megaBuffer.indexOf(',');
      int idx2 = megaBuffer.indexOf(',', idx1 + 1);
      int idx3 = megaBuffer.indexOf(',', idx2 + 1);
      
      if (idx1 > 0 && idx2 > 0 && idx3 > 0) {
        megaThrust1 = megaBuffer.substring(0, idx1).toFloat();
        megaTorque1 = megaBuffer.substring(idx1 + 1, idx2).toFloat();
        megaThrust2 = megaBuffer.substring(idx2 + 1, idx3).toFloat();
        megaTorque2 = megaBuffer.substring(idx3 + 1).toFloat();
      }
      megaBuffer = "";
    } else {
      megaBuffer += c;
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

    float valVolt1 = readINA226Voltage(0x44); 
    float valVolt2 = readINA226Voltage(0x41); 
    float valCurr1 = readACS724Current(pinCurr1);
    float valCurr2 = readACS724Current(pinCurr2);

    // Format: Thrust1, Torque1, RPM1, Volt1, Curr1, Thrust2, Torque2, RPM2, Volt2, Curr2
    Serial.print(megaThrust1, 3); 
    Serial.print(",");
    Serial.print(megaTorque1, 3); 
    Serial.print(",");
    Serial.print(currentRpm1, 0); 
    Serial.print(",");
    Serial.print(valVolt1, 2); 
    Serial.print(",");
    Serial.print(valCurr1, 2); 
    Serial.print(",");
    Serial.print(megaThrust2, 3); 
    Serial.print(",");
    Serial.print(megaTorque2, 3); 
    Serial.print(",");
    Serial.print(currentRpm2, 0); 
    Serial.print(",");
    Serial.print(valVolt2, 2);
    Serial.print(",");
    Serial.println(valCurr2, 2);                
  }
}