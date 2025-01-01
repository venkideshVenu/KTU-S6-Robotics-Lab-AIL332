# Arduino Serial Communication and LED Control

## Aim

To demonstrate serial communication between the Arduino and a computer, allowing control of an LED's state (ON/OFF) via the Serial Monitor.

---

## Components Required

- Arduino board (e.g., Arduino Uno)
- LED
- Resistor (220Ω - 1kΩ)
- Breadboard
- Jumper wires
- Computer with the Arduino IDE installed

---

## Program Overview

The program performs the following tasks:

1. Sets up serial communication at a baud rate of `9600`.
2. Reads input from the Serial Monitor.
3. Controls an LED based on the received input:
   - Sending `1` turns the LED ON.
   - Sending `0` turns the LED OFF.
4. Displays the received input in multiple formats (Decimal, Hexadecimal, and Character).

---

## Circuit Diagram

- Connect the **long leg (anode)** of the LED to pin `13` (or the pin defined in the code).
- Connect the **short leg (cathode)** of the LED to one end of the resistor.
- Connect the other end of the resistor to the **GND pin** on the Arduino board.

---
## Video Simulation

![](./serialmonitor.mp4)

---


## Program

### Code

```cpp
// Define the LED pin
const int led = 13; // You can change this to the pin connected to the LED

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
```

### Output

```text

--- Start Serial Monitor SEND_RCVE ---
Type in Box above:
(Decimal)(Hex)(Character)

1 31 1 LED ON 
0 30 0 LED OFF

```