#include <ESP32Servo.h>

Servo esc;              // Create a servo object to control the ESC
const int escPin = 26;  // GPIO pin connected to ESC signal wire

void setup() {
  // Initialize serial for debugging
  Serial.begin(115200);

  // Attach the ESC to the specified pin
  esc.attach(escPin, 1000, 2000);  // Min pulse width: 1000µs, Max pulse width: 2000µs

  // Initialize ESC: Set to neutral position
  esc.write(90);  // 90 maps to ~1500µs (neutral)
  delay(2000);    // Wait for ESC to initialize
}

void loop() {
  // Forward rotation (increase speed)
  Serial.println("Forward rotation");
  for (int angle = 90; angle <= 180; angle += 10) {  // 90 (1500µs) to 180 (2000µs)
    esc.write(angle);
    Serial.print("Angle: ");
    Serial.println(angle);
    delay(500);
  }

  // Stop motor
  Serial.println("Stopping");
  esc.write(90);  // Neutral position
  delay(2000);

  // Reverse rotation (increase speed)
  Serial.println("Reverse rotation");
  for (int angle = 90; angle >= 0; angle -= 10) {  // 90 (1500µs) to 0 (1000µs)
    esc.write(angle);
    Serial.print("Angle: ");
    Serial.println(angle);
    delay(500);
  }

  // Stop motor
  Serial.println("Stopping");
  esc.write(90);  // Neutral position
  delay(2000);
}