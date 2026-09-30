#include "HX711.h"
#include "Servo.h"
#include <Wire.h>

// Load Cells
HX711 thrustFront;
HX711 torqueFront;
HX711 thrustBack;
HX711 torqueBack;

// ESCs
Servo escFront;
Servo escBack;

// Pin Mappings
const int pinRpm1 = 2;        
const int pinRpm2 = 3;        
const int pinEsc1 = 9;        
const int pinEsc2 = 10;       
const int hxThrust1Dat = 4;   
const int hxThrust1Clk = 5;   
const int hxTorque1Dat = 6;   
const int hxTorque1Clk = 7;   
const int hxThrust2Dat = 8;   
const int hxThrust2Clk = 11;  
const int hxTorque2Dat = 12;  
const int hxTorque2Clk = A0;  

// Calibration & Safety Pins
const int pinEscCal = A3;      // Ground this pin during boot to run ESC Calibration

// System Constants
const float PULSES_PER_REV = 1.0;           
const float TORQUE_ARM_LENGTH_METERS = 1.0; 
const float ACS724_SENSITIVITY = 0.200;     // 200mV/A for ACS724-20AU unidirectional sensor

// RPM Tracking with Debounce (Period Measurement)
volatile unsigned long pulsePeriod1 = 0;
volatile unsigned long pulsePeriod2 = 0;
volatile unsigned long lastPulseTime1 = 0;
volatile unsigned long lastPulseTime2 = 0;
unsigned long lastSendTime = 0;

// Safety & Latching State
bool systemArmed = false;
unsigned long lastValidCmdMs = 0;
float currentOutputT1 = 0;
float currentOutputT2 = 0;

// Non-blocking Load Cell Cache
float valThrust1 = 0, valTorque1 = 0, valThrust2 = 0, valTorque2 = 0;

// Interrupt Service Routines for KE0068 (Active LOW on White)
void countRpm1() { 
  unsigned long now = micros();
  unsigned long interval = now - lastPulseTime1;
  // 5000us debounce caps the maximum readable RPM at 12,000, preventing noise double-triggers
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
// I2C Voltage Read Helper
// ==========================================================
float readINA226Voltage(uint8_t address) {
  Wire.beginTransmission(address);
  Wire.write(0x02); // Bus Voltage Register
  Wire.endTransmission();
  Wire.requestFrom(address, (uint8_t)2);
  if (Wire.available() >= 2) {
    uint16_t val = (Wire.read() << 8) | Wire.read();
    return val * 0.00125; // 1.25mV per LSB
  }
  return 0.0;
}

// ==========================================================
// ESC Throttle-Range Calibration
// ==========================================================
bool calibrationRequested() {
  pinMode(pinEscCal, INPUT_PULLUP);
  Serial.println("Ground Pin 8 within 3s to calibrate ESC...");
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
  Serial.println("Remove ground from Pin 8 to proceed to MIN.");
  
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
  Wire.setWireTimeout(25000, true); // Prevents I2C hangs from triggering the 500ms safety cutoff

  thrustFront.begin(hxThrust1Dat, hxThrust1Clk);
  torqueFront.begin(hxTorque1Dat, hxTorque1Clk);
  thrustBack.begin(hxThrust2Dat, hxThrust2Clk);
  torqueBack.begin(hxTorque2Dat, hxTorque2Clk);

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

  Serial.println("Dynamometer Hardware Initialized");
  lastValidCmdMs = millis();
}

void loop() {
  // Empty the entire serial buffer to prevent command lag and 500ms safety timeouts
  while (Serial.available() > 0) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    lastValidCmdMs = millis(); // Refresh safety timeout
    
    // PC Throttle commands map 0-100 % into 1100-1900 Microseconds with validation
    if (cmd.startsWith("T1_")) {
      float throttlePct = cmd.substring(3).toFloat();
      if (throttlePct < 0.0) throttlePct = 0.0;
      if (throttlePct > 100.0) throttlePct = 100.0;
      currentOutputT1 = throttlePct;
    } else if (cmd.startsWith("T2_")) {
      float throttlePct = cmd.substring(3).toFloat();
      if (throttlePct < 0.0) throttlePct = 0.0;
      if (throttlePct > 100.0) throttlePct = 100.0;
      currentOutputT2 = throttlePct;
    } 
    // Latching and Safety Commands
    else if (cmd == "UNLOCK_SINGLE" || cmd == "UNLOCK_CRP") {
      systemArmed = true;
    } else if (cmd == "LOCK_SINGLE" || cmd == "LOCK_CRP" || cmd == "STOP_ALL") {
      systemArmed = false;
      currentOutputT1 = 0.0;
      currentOutputT2 = 0.0;
    }
    // Manual Scale Application
    else if (cmd.startsWith("S_T1_")) { thrustFront.set_scale(cmd.substring(5).toFloat()); }
    else if (cmd.startsWith("S_Q1_")) { torqueFront.set_scale(cmd.substring(5).toFloat()); }
    else if (cmd.startsWith("S_T2_")) { thrustBack.set_scale(cmd.substring(5).toFloat()); }
    else if (cmd.startsWith("S_Q2_")) { torqueBack.set_scale(cmd.substring(5).toFloat()); }
    // Global Tare
    else if (cmd == "TARE_ALL") {
      thrustFront.tare(30); torqueFront.tare(30);
      thrustBack.tare(30); torqueBack.tare(30);
    }
    
    // Calibration 
    float calForceN = 4.903325; // Force of 500g mass
    float calTorqueNm = calForceN * TORQUE_ARM_LENGTH_METERS;

    if (cmd == "A") { thrustFront.tare(30); }
    else if (cmd == "B") { 
      thrustFront.set_scale(thrustFront.get_value(30) / calForceN); 
      Serial.print("CAL:T1:"); Serial.println(thrustFront.get_scale(), 4);
    } 
    else if (cmd == "C") { torqueFront.tare(30); }
    else if (cmd == "D") { 
      torqueFront.set_scale(torqueFront.get_value(30) / calTorqueNm); 
      Serial.print("CAL:Q1:"); Serial.println(torqueFront.get_scale(), 4);
    }
    else if (cmd == "E") { thrustBack.tare(30); }
    else if (cmd == "F") { 
      thrustBack.set_scale(thrustBack.get_value(30) / calForceN); 
      Serial.print("CAL:T2:"); Serial.println(thrustBack.get_scale(), 4);
    }
    else if (cmd == "G") { torqueBack.tare(30); }
    else if (cmd == "H") { 
      torqueBack.set_scale(torqueBack.get_value(30) / calTorqueNm); 
      Serial.print("CAL:Q2:"); Serial.println(torqueBack.get_scale(), 4);
    }
  }

  // --- HARDWARE SAFETY OVERRIDE ---
  if (!systemArmed || (millis() - lastValidCmdMs > 500)) {
    escFront.writeMicroseconds(1100);
    escBack.writeMicroseconds(1100);
  } else {
    escFront.writeMicroseconds(1100 + (currentOutputT1 * 8.0));
    escBack.writeMicroseconds(1100 + (currentOutputT2 * 8.0));
  }

  // --- SENSOR POLLING (10Hz) ---
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

    if (thrustFront.is_ready()) valThrust1 = thrustFront.get_units(1);
    if (torqueFront.is_ready()) valTorque1 = torqueFront.get_units(1);
    if (thrustBack.is_ready())  valThrust2 = thrustBack.get_units(1);
    if (torqueBack.is_ready())  valTorque2 = torqueBack.get_units(1);

    // INA226 Voltage (I2C)
    float valVolt1 = readINA226Voltage(0x40);
    float valVolt2 = readINA226Voltage(0x41);

    // ACS724 Current (Analog) - Unidirectional Offset (0.5V)
    float rawA1 = analogRead(A1) * (5.0 / 1023.0);
    float valCurr1 = abs((rawA1 - 0.5) / ACS724_SENSITIVITY);
    
    float rawA2 = analogRead(A2) * (5.0 / 1023.0);
    float valCurr2 = abs((rawA2 - 0.5) / ACS724_SENSITIVITY);

    // Send formatted string to GUI
    Serial.print(valThrust1, 3); Serial.print(",");
    Serial.print(valTorque1, 3); Serial.print(",");
    Serial.print(currentRpm1, 0); Serial.print(",");
    Serial.print(valVolt1, 2); Serial.print(","); 
    Serial.print(valCurr1, 2); Serial.print(","); 
    Serial.print(valThrust2, 3); Serial.print(",");
    Serial.print(valTorque2, 3); Serial.print(",");
    Serial.print(currentRpm2, 0); Serial.print(",");
    Serial.print(valVolt2, 2); Serial.print(","); 
    Serial.println(valCurr2, 2);                  
  }
}