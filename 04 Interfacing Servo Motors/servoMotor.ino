#include <Servo.h>

int servoPin = 9; // Servo control pin
Servo servo; // Create a servo object
int angle = 0; // Servo position in degrees

void setup() {
  servo.attach(servoPin); // Attach the servo to pin 9
}

void loop() {
  // Sweep from 0 to 180 degrees
  for (angle = 0; angle <= 180; angle++) {
    servo.write(angle); // Set servo to current angle
    delay(15); // Wait for servo to reach position
  }

  // Sweep back from 180 to 0 degrees
  for (angle = 180; angle >= 0; angle--) {
    servo.write(angle); // Set servo to current angle
    delay(15); // Wait for servo to reach position
  }
}