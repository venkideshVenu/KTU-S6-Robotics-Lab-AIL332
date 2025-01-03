// Define pin connections
const int irSensorPin = 2;   // Pin connected to the IR sensor
const int ledPin = 12;       // Pin connected to the LED

void setup() {
  // Initialize the IR sensor pin as input
  pinMode(irSensorPin, INPUT);

  // Initialize the LED pin as output
  pinMode(ledPin, OUTPUT);
  
  Serial.begin(9600);

  // Ensure the LED is off at the start
  digitalWrite(ledPin, LOW);
}

void loop() {
  // Read the IR sensor value
  int irValue = digitalRead(irSensorPin);

  // Check if an obstacle is detected (usually HIGH for detection)
  if (irValue == LOW) {
    // Turn on the LED
    digitalWrite(ledPin, HIGH);
    Serial.println("Obstacle Detected !!!");

    // Wait for 500 milliseconds
    delay(500);

    // Turn off the LED
    digitalWrite(ledPin, LOW);
  }

  // Small delay to avoid rapid toggling
  delay(100);
}