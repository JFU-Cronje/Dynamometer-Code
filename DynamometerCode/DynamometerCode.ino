#include "HX711.h"
#include "Servo.h"

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

// System Constants
const float PULSES_PER_REV = 1.0;           // 1 white sticker per rotor
const float TORQUE_ARM_LENGTH_METERS = 1.0; // IMPORTANT: Replace 1.0 with your actual torque arm length in meters

// RPM Tracking with Debounce
volatile unsigned long rpmPulses1 = 0;
volatile unsigned long rpmPulses2 = 0;
volatile unsigned long lastPulseTime1 = 0;
volatile unsigned long lastPulseTime2 = 0;
unsigned long lastSendTime = 0;

// Non-blocking Load Cell Cache
float valThrust1 = 0, valTorque1 = 0, valThrust2 = 0, valTorque2 = 0;

// Interrupt Service Routines for KE0068 (Active LOW on White)
void countRpm1() { 
  unsigned long now = micros();
  // 1000us debounce caps the maximum readable RPM at 60,000, preventing noise double-triggers
  if (now - lastPulseTime1 > 1000) {
    rpmPulses1++; 
    lastPulseTime1 = now;
  }
}

void countRpm2() { 
  unsigned long now = micros();
  if (now - lastPulseTime2 > 1000) {
    rpmPulses2++; 
    lastPulseTime2 = now;
  }
}

void setup() {
  Serial.begin(115200);

  // Initialize HX711s
  thrustFront.begin(hxThrust1Dat, hxThrust1Clk);
  torqueFront.begin(hxTorque1Dat, hxTorque1Clk);
  thrustBack.begin(hxThrust2Dat, hxThrust2Clk);
  torqueBack.begin(hxTorque2Dat, hxTorque2Clk);

  // Initialize ESCs
  escFront.attach(pinEsc1, 1000, 2000);
  escBack.attach(pinEsc2, 1000, 2000);
  escFront.writeMicroseconds(1000);
  escBack.writeMicroseconds(1000);

  // Initialize RPM Interrupts for FALLING edge (HIGH to LOW transition)
  pinMode(pinRpm1, INPUT_PULLUP);
  pinMode(pinRpm2, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(pinRpm1), countRpm1, FALLING);
  attachInterrupt(digitalPinToInterrupt(pinRpm2), countRpm2, FALLING);

  Serial.println("Dynamometer Hardware Initialized");
}

void loop() {
  // 1. Listen for commands from the PC GUI
  if (Serial.available() > 0) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    
    // ESC Control
    if (cmd.startsWith("T1_")) {
      float throttle = cmd.substring(3).toFloat();
      escFront.writeMicroseconds(1000 + (throttle * 1000));
    } else if (cmd.startsWith("T2_")) {
      float throttle = cmd.substring(3).toFloat();
      escBack.writeMicroseconds(1000 + (throttle * 1000));
    } else if (cmd == "STOP_ALL") {
      escFront.writeMicroseconds(1000);
      escBack.writeMicroseconds(1000);
    }
    
    // Calibration (Taking 30 samples for a highly stable average)
    float calForceN = 4.903325; // Force of 500g mass
    float calTorqueNm = calForceN * TORQUE_ARM_LENGTH_METERS;

    if (cmd == "A") { torqueFront.tare(30); }
    else if (cmd == "B") { torqueFront.set_scale(torqueFront.get_value(30) / calTorqueNm); } 
    else if (cmd == "C") { thrustFront.tare(30); }
    else if (cmd == "D") { thrustFront.set_scale(thrustFront.get_value(30) / calForceN); }
    else if (cmd == "E") { torqueBack.tare(30); }
    else if (cmd == "F") { torqueBack.set_scale(torqueBack.get_value(30) / calTorqueNm); }
    else if (cmd == "G") { thrustBack.tare(30); }
    else if (cmd == "H") { thrustBack.set_scale(thrustBack.get_value(30) / calForceN); }
  }

  // 2. Send real data back to the PC every 100ms (10Hz)
  unsigned long now = millis();
  if (now - lastSendTime >= 100) {
    unsigned long dt = now - lastSendTime;
    lastSendTime = now;

    // Safely copy RPM pulses
    noInterrupts(); 
    unsigned long pulses1 = rpmPulses1;
    unsigned long pulses2 = rpmPulses2;
    rpmPulses1 = 0;
    rpmPulses2 = 0;
    interrupts();

    float currentRpm1 = (pulses1 / PULSES_PER_REV * 60000.0) / dt;
    float currentRpm2 = (pulses2 / PULSES_PER_REV * 60000.0) / dt;

    // Read Load Cells Non-Blocking
    if (thrustFront.is_ready()) valThrust1 = thrustFront.get_units(1);
    if (torqueFront.is_ready()) valTorque1 = torqueFront.get_units(1);
    if (thrustBack.is_ready())  valThrust2 = thrustBack.get_units(1);
    if (torqueBack.is_ready())  valTorque2 = torqueBack.get_units(1);

    // Format: Thrust1,Torque1,RPM1,Volt1,Curr1,Thrust2,Torque2,RPM2,Volt2,Curr2
    Serial.print(valThrust1, 3); Serial.print(",");
    Serial.print(valTorque1, 3); Serial.print(",");
    Serial.print(currentRpm1, 0); Serial.print(",");
    Serial.print(0.00); Serial.print(","); 
    Serial.print(0.00); Serial.print(","); 
    Serial.print(valThrust2, 3); Serial.print(",");
    Serial.print(valTorque2, 3); Serial.print(",");
    Serial.print(currentRpm2, 0); Serial.print(",");
    Serial.print(0.00); Serial.print(","); 
    Serial.println(0.00);                  
  }
}