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
const float PULSES_PER_REV = 1.0;          // Adjust if your RPM sensor triggers multiple times per revolution
const float TORQUE_ARM_LENGTH_METERS = 1.0; // IMPORTANT: Replace 1.0 with the physical length of your torque arm in meters

// RPM Tracking
volatile unsigned long rpmPulses1 = 0;
volatile unsigned long rpmPulses2 = 0;
unsigned long lastTime = 0;

// Non-blocking Load Cell Cache
float valThrust1 = 0, valTorque1 = 0, valThrust2 = 0, valTorque2 = 0;

void countRpm1() { rpmPulses1++; }
void countRpm2() { rpmPulses2++; }

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

  // Initialize RPM Interrupts
  pinMode(pinRpm1, INPUT_PULLUP);
  pinMode(pinRpm2, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(pinRpm1), countRpm1, RISING);
  attachInterrupt(digitalPinToInterrupt(pinRpm2), countRpm2, RISING);

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
    
    // Calibration 
    // A 500g weight applies a downward force of 4.903325 Newtons.
    // For torque, the true moment is Force * Lever Arm.
    float calForceN = 4.903325;
    float calTorqueNm = calForceN * TORQUE_ARM_LENGTH_METERS;

    if (cmd == "A") { torqueFront.tare(10); }
    else if (cmd == "B") { torqueFront.set_scale(torqueFront.get_value(10) / calTorqueNm); } 
    else if (cmd == "C") { thrustFront.tare(10); }
    else if (cmd == "D") { thrustFront.set_scale(thrustFront.get_value(10) / calForceN); }
    else if (cmd == "E") { torqueBack.tare(10); }
    else if (cmd == "F") { torqueBack.set_scale(torqueBack.get_value(10) / calTorqueNm); }
    else if (cmd == "G") { thrustBack.tare(10); }
    else if (cmd == "H") { thrustBack.set_scale(thrustBack.get_value(10) / calForceN); }
  }

  // 2. Send real data back to the PC every 100ms (10Hz)
  unsigned long now = millis();
  if (now - lastTime >= 100) {
    unsigned long dt = now - lastTime;
    lastTime = now;

    // RPM Calculation
    noInterrupts(); // Briefly pause interrupts to prevent data corruption while copying
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
    Serial.print(0.00); Serial.print(","); // Volt 1 (Not wired)
    Serial.print(0.00); Serial.print(","); // Curr 1 (Not wired)
    Serial.print(valThrust2, 3); Serial.print(",");
    Serial.print(valTorque2, 3); Serial.print(",");
    Serial.print(currentRpm2, 0); Serial.print(",");
    Serial.print(0.00); Serial.print(","); // Volt 2 (Not wired)
    Serial.println(0.00);                  // Curr 2 (Not wired)
  }
}