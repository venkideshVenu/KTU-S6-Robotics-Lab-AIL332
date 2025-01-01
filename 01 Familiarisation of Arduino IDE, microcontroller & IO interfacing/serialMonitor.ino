// Define the LED pin
const int led = 12; // You can change this to the pin connected to the LED

// Variable to store received byte
char ByteReceived;

void setup() {
  // Start serial communication
  Serial.begin(9600);
  Serial.println("--- Start Serial Monitor SEND_RCVE ---");
  Serial.println("Type in Box above:");
  Serial.println("(Decimal)(Hex)(Character)");
  Serial.println();

  // Set the LED pin as an output
  pinMode(led, OUTPUT);
}

void loop() {
  // Check if data is available in the serial buffer
  if (Serial.available() > 0) {
    // Read the incoming byte
    ByteReceived = Serial.read();

    // Print the byte in decimal, hex, and as a character
    Serial.print(ByteReceived);         // Decimal
    Serial.print(" ");
    Serial.print(ByteReceived, HEX);    // Hexadecimal
    Serial.print(" ");
    Serial.print(char(ByteReceived));   // Character

    // Turn LED ON or OFF based on received character
    if (ByteReceived == '1') { // Single Quote for character comparison
      digitalWrite(led, HIGH); // Turn LED ON
      Serial.print(" LED ON ");
    }
    if (ByteReceived == '0') {
      digitalWrite(led, LOW); // Turn LED OFF
      Serial.print(" LED OFF");
    }

    Serial.println(); // End the line
  }
}
