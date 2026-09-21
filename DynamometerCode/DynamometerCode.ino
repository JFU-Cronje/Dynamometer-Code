#include "HX711.h"

// Define Load Cell Objects
HX711 thrustFront;
HX711 torqueFront;
HX711 thrustBack;
HX711 torqueBack;

void setup() {
  Serial.begin(115200);
  
  // Initialize pins (DOUT, SCK)
  thrustFront.begin(2, 3);
  torqueFront.begin(4, 5);
  thrustBack.begin(6, 7);
  torqueBack.begin(8, 9);
  
  Serial.println("Dynamometer Load Cells Initialized");
}

void loop() {
  // Listen for commands from the PC GUI
  if (Serial.available() > 0) {
    char cmd = Serial.read();
    
    switch (cmd) {
      // 1. TARE TORQUE: PC sends 'A' before attaching the 500g weight
      case 'A':
        torqueFront.tare(20);
        Serial.println("Front Torque Tared.");
        break;
        
      // 2. CALIBRATE TORQUE: PC sends 'B' after attaching the 500g weight
      case 'B':
        long rawTorque = torqueFront.get_value(20);
        // Calculate the scale factor for 500g
        torqueFront.set_scale(rawTorque / 500.0); 
        Serial.println("Front Torque Calibrated to 500g.");
        break;
        
      // 3. TARE THRUST: PC sends 'C' with stand laid flat, no weight
      case 'C':
        thrustFront.tare(20);
        Serial.println("Front Thrust Tared.");
        break;
        
      // 4. CALIBRATE THRUST: PC sends 'D' with stand flat, 500g weight added
      case 'D':
        long rawThrust = thrustFront.get_value(20);
        thrustFront.set_scale(rawThrust / 500.0);
        Serial.println("Front Thrust Calibrated to 500g.");
        break;
        
      // 5. PRINT DATA: PC sends 'P' to request a single reading of all sensors
      case 'P':
        Serial.print(thrustFront.get_units(5));
        Serial.print(",");
        Serial.print(torqueFront.get_units(5));
        Serial.print(",");
        Serial.print(thrustBack.get_units(5));
        Serial.print(",");
        Serial.println(torqueBack.get_units(5));
        break;
    }
  }
}