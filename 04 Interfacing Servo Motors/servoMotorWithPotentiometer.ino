#include <Servo.h>

int potPin = A0; // Potentiometer connected to Analog pin A0
int servoPin = 9; // Servo control pin
Servo servo; // Create a servo object

void setup() {
  servo.attach(servoPin); // Attach the servo to pin 9
}

void loop() {
  int reading = analogRead(potPin); // Read potentiometer value (0 to 1023)
  int angle = map(reading, 0, 1023, 0, 180); // Map to servo angle (0 to 180 degrees)
  servo.write(angle); // Set servo to calculated angle
  delay(15); // Wait for servo to reach position
}
