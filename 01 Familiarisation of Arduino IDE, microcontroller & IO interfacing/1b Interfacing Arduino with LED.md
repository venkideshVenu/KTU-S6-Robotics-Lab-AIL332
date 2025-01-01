# Interfacing Arduino  with LED

## Aim
To create Arduino programs that blink both an internal LED and an external LED at regular intervals.

---

## Components Required
### For External LED Blink:
- Arduino board (e.g., Arduino Uno)
- LED
- Resistor (220Ω - 1kΩ) (optional)
- Breadboard
- Jumper wires

---

### For Internal LED Blink:
- Arduino board with a built-in LED (e.g., Arduino Uno)

---

## Video Simulation
![My Image](./led.gif)

---

## Programs

### 1. Internal LED Blink
This program blinks the onboard LED on the Arduino board, which is usually connected to the `LED_BUILTIN` pin.

#### Code
```cpp
void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
}

void loop() {
  digitalWrite(LED_BUILTIN, HIGH);
  delay(1000); // Wait for 1 second
  digitalWrite(LED_BUILTIN, LOW);
  delay(1000); // Wait for 1 second
}
```


### 2. External LED Blink
This program blinks the external LED connected to pin 13 on the Arduino board, which is usually connected to the `LED_BUILTIN` pin.

#### Code
```cpp
// Define the LED pin
const int ledPin = 13; // Most Arduino boards have an onboard LED on pin 13

void setup() {
  // Set the LED pin as an output
  pinMode(ledPin, OUTPUT);
}

void loop() {
  // Turn the LED on
  digitalWrite(ledPin, HIGH);
  delay(1000); // Wait for 1 second (1000 ms)
  
  // Turn the LED off
  digitalWrite(ledPin, LOW);
  delay(1000); // Wait for 1 second (1000 ms)
}

```

---

## Notes
- You can adjust the delay interval in the programs to modify the blink speed.
- Ensure proper connections to avoid damage to the LED or Arduino board.
