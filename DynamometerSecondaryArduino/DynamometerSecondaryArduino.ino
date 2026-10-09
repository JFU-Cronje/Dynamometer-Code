#include "HX711.h"

// Mega Pins for 4 Load Cells
const int pinThrust1_DT = 4;
const int pinThrust1_SCK = 5;
const int pinTorque1_DT = 6;
const int pinTorque1_SCK = 7;
const int pinThrust2_DT = 8;
const int pinThrust2_SCK = 9;
const int pinTorque2_DT = 10;
const int pinTorque2_SCK = 11;

// Load Cell Objects
HX711 loadCellThrust1;
HX711 loadCellTorque1;
HX711 loadCellThrust2;
HX711 loadCellTorque2;

void setup() {
  // Serial1 sends data directly to the Main Arduino Uno 
  // (Mega TX1 Pin 18 connects to Uno RX Pin 4)
  Serial1.begin(38400);

  loadCellThrust1.begin(pinThrust1_DT, pinThrust1_SCK);
  loadCellTorque1.begin(pinTorque1_DT, pinTorque1_SCK);
  loadCellThrust2.begin(pinThrust2_DT, pinThrust2_SCK);
  loadCellTorque2.begin(pinTorque2_DT, pinTorque2_SCK);
}

void loop() {
  float valThrust1 = 0.0;
  float valTorque1 = 0.0;
  float valThrust2 = 0.0;
  float valTorque2 = 0.0;
  
  // Read sensors
  if (loadCellThrust1.is_ready()) valThrust1 = loadCellThrust1.get_units(1);
  if (loadCellTorque1.is_ready()) valTorque1 = loadCellTorque1.get_units(1);
  if (loadCellThrust2.is_ready()) valThrust2 = loadCellThrust2.get_units(1);
  if (loadCellTorque2.is_ready()) valTorque2 = loadCellTorque2.get_units(1);

  // Format: Thrust1,Torque1,Thrust2,Torque2
  String dataString = String(valThrust1, 3) + "," + 
                      String(valTorque1, 3) + "," + 
                      String(valThrust2, 3) + "," + 
                      String(valTorque2, 3);
  
  // Transmit to Uno via Hardware Serial1
  Serial1.println(dataString);
  
  delay(20); // Poll at 50Hz to ensure the Uno's 10Hz loop always has fresh data
}