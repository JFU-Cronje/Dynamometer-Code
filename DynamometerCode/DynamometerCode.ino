float throttle1 = 0.0;
float throttle2 = 0.0;
unsigned long lastSendTime = 0;

void setup() {
  Serial.begin(115200);
  Serial.println("Dynamometer Simulation Initialized");
}

void loop() {
  // 1. Listen for commands from the PC GUI
  if (Serial.available() > 0) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    
    // Parse the throttle slider values sent from C#
    if (cmd.startsWith("T1_")) {
      throttle1 = cmd.substring(3).toFloat();
    } else if (cmd.startsWith("T2_")) {
      throttle2 = cmd.substring(3).toFloat();
    } else if (cmd == "STOP_ALL") {
      throttle1 = 0.0;
      throttle2 = 0.0;
    }
  }

  // 2. Send simulated data back to the PC every 100ms
  if (millis() - lastSendTime >= 100) {
    lastSendTime = millis();

    // Generate fake data mapped to the current throttle position
    float thrust1 = (throttle1 * 100.0) + (random(-10, 10) / 100.0);
    float torque1 = (throttle1 * 10.0) + (random(-5, 5) / 100.0);
    float rpm1    = (throttle1 * 50000.0) + random(-20, 20);
    float volt1   = 24.0 - (throttle1 * 5.0) + (random(-5, 5) / 100.0);
    float curr1   = (throttle1 * 800.0) + (random(-50, 50) / 100.0);

    float thrust2 = (throttle2 * 100.0) + (random(-10, 10) / 100.0);
    float torque2 = (throttle2 * 10.0) + (random(-5, 5) / 100.0);
    float rpm2    = (throttle2 * 50000.0) + random(-20, 20);
    float volt2   = 24.0 - (throttle2 * 5.0) + (random(-5, 5) / 100.0);
    float curr2   = (throttle2 * 800.0) + (random(-50, 50) / 100.0);

    // Flatline the data if throttle is exactly 0
    if (throttle1 <= 0) { thrust1 = 0; torque1 = 0; rpm1 = 0; curr1 = 0; volt1 = 24.0; }
    if (throttle2 <= 0) { thrust2 = 0; torque2 = 0; rpm2 = 0; curr2 = 0; volt2 = 24.0; }

    // Format: Thrust1,Torque1,RPM1,Volt1,Curr1,Thrust2,Torque2,RPM2,Volt2,Curr2
    Serial.print(thrust1); Serial.print(",");
    Serial.print(torque1); Serial.print(",");
    Serial.print(rpm1); Serial.print(",");
    Serial.print(volt1); Serial.print(",");
    Serial.print(curr1); Serial.print(",");
    Serial.print(thrust2); Serial.print(",");
    Serial.print(torque2); Serial.print(",");
    Serial.print(rpm2); Serial.print(",");
    Serial.print(volt2); Serial.print(",");
    Serial.println(curr2);
  }
}