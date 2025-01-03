// Define pin connections for the ultrasonic sensor
const int trigPin = 9;  // Pin connected to the Trig pin of the sensor
const int echoPin = 10; // Pin connected to the Echo pin of the sensor

// Variable to store the calculated distance
long duration;
float distance;

void setup() {
  // Initialize the serial communication for debugging
  Serial.begin(9600);

  // Set the trigPin as an output and echoPin as an input
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
}

void loop() {
  // Clear the trigPin
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);

  // Send a 10-microsecond pulse to the trigPin
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // Read the echoPin and calculate the duration (time for the sound wave to return)
  duration = pulseIn(echoPin, HIGH);

  // Calculate the distance in cm
  // Speed of sound in air is approximately 343 m/s (or 34300 cm/s)
  // Divide by 2 as the wave travels to the object and back
  distance = (duration * 0.0343) / 2;

  // Print the distance to the Serial Monitor
  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.println(" cm");

  // Wait for a short period before the next measurement
  delay(500);
}
