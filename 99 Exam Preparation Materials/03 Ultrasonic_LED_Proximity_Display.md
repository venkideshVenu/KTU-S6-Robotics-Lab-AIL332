
# 🔬 Experiment: Ultrasonic Sensor-Based LED Proximity Display

---

## 🎯 **Objective**
To build a system that detects the distance of nearby objects using an **ultrasonic sensor (HC-SR04)** and displays the proximity visually using **4 LEDs**.

---

## 🧰 **Requirements**

### 🛠 Hardware
- Arduino Uno R3
- Ultrasonic Sensor (HC-SR04)
- 4 LEDs
- 4 Resistors (220Ω)
- Breadboard and jumper wires
- USB cable for Arduino

### 💻 Software
- Arduino IDE (with COM port and board set properly)

### 📚 Knowledge
- Basic understanding of Arduino
- Familiarity with digital I/O pins
- Understanding of ultrasonic distance measurement

---

## 📖 **Theory**
The HC-SR04 ultrasonic sensor uses sound waves to measure distance. It emits a pulse through the **Trig** pin and listens for the echo through the **Echo** pin. The time between sending and receiving the pulse is used to calculate the distance:

\[
\text{Distance (cm)} = \frac{\text{Time (µs)} \times 0.034}{2}
\]

Based on this measured distance, different LEDs are turned ON to represent how close an object is to the sensor.

---

## 🧪 **Procedure**

1. **Connect** the ultrasonic sensor to the Arduino:
   - Trig → Digital Pin 10
   - Echo → Digital Pin 9
2. **Connect** the LEDs to pins 7, 6, 5, and 4 respectively via 220Ω resistors.
3. **Upload** the code (given below) to your Arduino Uno.
4. **Observe** the LEDs light up as you bring an object closer to the sensor.

---
## Video Simulation


![](./led%20proximity%20display.gif)

---
## 💻 **Code**

```cpp
// Define ultrasonic sensor pins
const int trig = 10;
const int echo = 9;

// Define LED pins
const int led1 = 7;
const int led2 = 6;
const int led3 = 5;
const int led4 = 4;

void setup()
{
  // Set trig as output and echo as input
  pinMode(trig, OUTPUT);
  pinMode(echo, INPUT);
  
  // Set LED pins as output
  pinMode(led1, OUTPUT);
  pinMode(led2, OUTPUT);
  pinMode(led3, OUTPUT);
  pinMode(led4, OUTPUT);

  // Start serial communication
  Serial.begin(9600);
}

void loop()
{
  // Trigger the ultrasonic pulse
  digitalWrite(trig, LOW);
  delayMicroseconds(2);
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig,LOW);
  
  // Measure duration of echo
  float duration = pulseIn(echo, HIGH);
  
  // Calculate distance in cm
  float distance = duration * 0.034 / 2;
  Serial.println(distance); // Output to serial monitor
  
  // Light up LEDs based on distance range
  if (distance < 50){
    digitalWrite(led1, HIGH);
    digitalWrite(led2, HIGH);
    digitalWrite(led3, HIGH);
    digitalWrite(led4, HIGH);
  }
  else if (distance < 100){
    digitalWrite(led1, HIGH);
    digitalWrite(led2, HIGH);
    digitalWrite(led3, HIGH);
    digitalWrite(led4, LOW);
  }
  else if (distance < 150){
    digitalWrite(led1, HIGH);
    digitalWrite(led2, HIGH);
    digitalWrite(led3, LOW);
    digitalWrite(led4, LOW);
  }
  else if (distance < 200){
    digitalWrite(led1, HIGH);
    digitalWrite(led2, LOW);
    digitalWrite(led3, LOW);
    digitalWrite(led4, LOW);
  }
  else {
    // Turn off all LEDs for distance >= 200 cm
    digitalWrite(led1, LOW);
    digitalWrite(led2, LOW);
    digitalWrite(led3, LOW);
    digitalWrite(led4, LOW);
  }

  // Wait before next reading
  delay(500);
}
```

---

## 🖥️ **Expected Output**

| Distance (cm) | LED1 | LED2 | LED3 | LED4 |
|---------------|------|------|------|------|
| < 50          | ON   | ON   | ON   | ON   |
| 50 - 99       | ON   | ON   | ON   | OFF  |
| 100 - 149     | ON   | ON   | OFF  | OFF  |
| 150 - 199     | ON   | OFF  | OFF  | OFF  |
| ≥ 200         | OFF  | OFF  | OFF  | OFF  |

Additionally, the distance (in cm) will be displayed on the **Serial Monitor** every 0.5 seconds.

---

## ✅ **Conclusion**

This experiment successfully demonstrates how to:
- Use an **ultrasonic sensor** to measure distances
- Convert time-of-flight to distance using basic math
- Use **LEDs as indicators** to visualize proximity levels
- Interface multiple components with the Arduino Uno

---

## 📝 **Additional Notes**

- Ensure the **Echo pin** is connected to a digital input pin only.
- Use appropriate resistors for LEDs to avoid burning them out.
- Sensor readings might fluctuate slightly due to ambient noise—averaging values can improve accuracy.
- Make sure the object being measured has a flat surface for accurate reflections.

---
