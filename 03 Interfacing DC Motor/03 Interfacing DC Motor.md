# **Experiment No: 3**  
## **Interfacing DC Motors with Arduino - Speed and Direction Control**  

---

## **Aim**  
To write a program to generate PWM pulses for speed control of a DC motor.

---

## **Objective**  
To control the speed and direction of rotation of a DC motor using an Arduino UNO board.

---

## **Components Required**  
1. **Arduino UNO** – 1 unit  
2. **DC Motor** – 1 unit  
3. **L298N or L293D Motor Driver** – 1 unit  
4. **Connecting Wires**  
5. **Breadboard** – 1 unit  
6. **External Power Supply** – 1 unit  



---

## **Overview of DC Motor**  
- **Working Principle**: Converts electrical energy into mechanical energy.
- **Components**: Stator (magnets and brushes) and Rotor (shaft, windings, commutator).
- **Operation**: Rotation occurs when magnetic poles attract/repel each other.

---

## **Controlling Motor Speed**  
- **Pulse Width Modulation (PWM)** regulates average voltage by switching ON/OFF.
- **Duty Cycle**: 
  - 100% = Full voltage
  - 50% = Half voltage
  - 0% = No voltage

---

## **H-Bridge for Direction Control**  
- **H-Bridge**: Circuit that controls motor direction.
- Consists of four switches and protection diodes.
- Reverses current flow to change motor direction.

---

## **L298N Motor Driver**

### **Circuit Connections**

1. **Power Connections**:
   - Connect power supply to L298N
   - Connect GND of Arduino to GND of L298N

2. **Logic Connections**:
   - ENA (PWM) → Arduino pin 9
   - IN1 → Arduino pin 8
   - IN2 → Arduino pin 7

3. **Motor Connections**:
   - Motor → OUT1 and OUT2

### **Simple L298N Program**

```cpp
// Simple DC Motor Control with L298N

// Pin definitions
const int ENA = 9;  // PWM pin for speed control
const int IN1 = 8;  // Direction control 1
const int IN2 = 7;  // Direction control 2

void setup() {
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
}

void loop() {
  // Clockwise rotation at full speed
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, 255);
  delay(3000);  // Run for 3 seconds
  
  // Stop motor
  analogWrite(ENA, 0);
  delay(1000);  // Stop for 1 second
  
  // Counterclockwise rotation at half speed
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  analogWrite(ENA, 128);
  delay(3000);  // Run for 3 seconds
  
  // Stop motor
  analogWrite(ENA, 0);
  delay(1000);  // Stop for 1 second
}
```

---

## **L293D Motor Driver**

### **Circuit Connections**

1. **Power Connections**:
   - Pin 16 (VCC1) → 5V from Arduino
   - Pin 8 (VCC2) → External power supply
   - Pins 4, 5, 12, 13 (GND) → GND

2. **Logic Connections**:
   - Pin 1 (Enable) → Arduino pin 9
   - Pin 2 (Input 1) → Arduino pin 8
   - Pin 7 (Input 2) → Arduino pin 7

3. **Motor Connections**:
   - Motor → Pin 3 (Output 1) and Pin 6 (Output 2)

---

## **Video Simulation**

![](./DCMotor.gif)

---
### **Simple L293D Program**

```cpp
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
```

---

## **Comparison: L298N vs L293D**

| Feature | L298N | L293D |
|---------|-------|-------|
| **Max Current** | 2A per channel | 600mA per channel |
| **Best For** | Larger motors | Small motors |

---

## **Troubleshooting Tips**

1. **Motor Not Working**:
   - Check connections
   - Verify power supply
   - Ensure code is uploaded correctly

2. **Motor Too Slow**:
   - Increase PWM value
   - Check power supply voltage

---

## **Safety Notes**

1. Always disconnect power when changing connections
2. Avoid overloading the motor driver
3. Use appropriate voltage for your motor

---

## **Conclusion**  
This experiment demonstrates how to control DC motor speed and direction using Arduino with either L298N or L293D motor drivers. PWM is used for speed control, while H-bridge configuration enables direction control.

---

## **References**
1. Arduino PWM: [https://www.arduino.cc/reference/en/](https://www.arduino.cc/reference/en/)
2. L298N Datasheet: [https://www.st.com/resource/en/datasheet/l298.pdf](https://www.st.com/resource/en/datasheet/l298.pdf)
3. L293D Datasheet: [https://www.ti.com/lit/ds/symlink/l293d.pdf](https://www.ti.com/lit/ds/symlink/l293d.pdf)