const int pwm = 2;    // PWM pin for speed control
const int in_1 = 8;   // Logic pin 1 for direction control
const int in_2 = 9;   // Logic pin 2 for direction control

void setup() {
  pinMode(pwm, OUTPUT);  // Set PWM pin as output
  pinMode(in_1, OUTPUT); // Set direction pin 1 as output
  pinMode(in_2, OUTPUT); // Set direction pin 2 as output
}

void loop() {
  // Clockwise Rotation: in_1 = HIGH, in_2 = LOW
  digitalWrite(in_1, HIGH);
  digitalWrite(in_2, LOW);
  analogWrite(pwm, 255); // Maximum speed
  delay(3000);           // Rotate clockwise for 3 seconds

  // Brake: in_1 = HIGH, in_2 = HIGH
  digitalWrite(in_1, HIGH);
  digitalWrite(in_2, HIGH);
  delay(1000);           // Stop motor for 1 second

  // Counterclockwise Rotation: in_1 = LOW, in_2 = HIGH
  digitalWrite(in_1, LOW);
  digitalWrite(in_2, HIGH);
  analogWrite(pwm, 255); // Maximum speed
  delay(3000);           // Rotate counterclockwise for 3 seconds

  // Brake: in_1 = HIGH, in_2 = HIGH
  digitalWrite(in_1, HIGH);
  digitalWrite(in_2, HIGH);
  delay(1000);           // Stop motor for 1 second
}