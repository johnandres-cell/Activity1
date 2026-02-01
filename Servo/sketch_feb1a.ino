#include <ESP32Servo.h>

Servo myServo;

// -----------------------
// CONFIGURATION
// -----------------------
const int SERVO_PIN = 13;     // GPIO pin connected to servo signal
int servoPos = 90;            // Start at middle position (90°)

// -----------------------
// SETUP
// -----------------------
void setup() {
  Serial.begin(115200);
  delay(1000);

  // Servo setup
  myServo.setPeriodHertz(50);          // Standard servo frequency
  myServo.attach(SERVO_PIN, 500, 2400); // Min/Max pulse width
  myServo.write(servoPos);

  Serial.println("ESP32 Gesture Servo Ready");
}

// -----------------------
// LOOP
// -----------------------
void loop() {
  if (Serial.available() > 0) {
    // Read angle sent from Python (e.g. "90\n")
    int newPos = Serial.parseInt();

    // Clear serial buffer
    while (Serial.available()) Serial.read();

    // Validate and move servo
    if (newPos >= 0 && newPos <= 180) {
      servoPos = newPos;
      myServo.write(servoPos);

      Serial.print("Servo Position: ");
      Serial.println(servoPos);
    } else {
      Serial.print("Invalid angle received: ");
      Serial.println(newPos);
    }
  }
}
