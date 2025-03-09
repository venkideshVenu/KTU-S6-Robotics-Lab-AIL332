# Obstacle Avoidance Robot

## Aim  
To design and implement an obstacle avoidance system for a mobile robot using an ultrasonic sensor to detect and avoid obstacles while navigating towards a predefined target point.

---

## Components Required  
- DC Motors  
- Motor Driver (L298N)  
- Batteries  
- Arduino Uno Board  
- Jumper Wires  
- Breadboard  
- Wheels  
- Chassis  
- Ultrasonic Sensor (HC-SR04)  

---

## Circuit Description  

### Motor Driver (L298N) Connection  
- **IN1 (L298N) → Pin 8 (Arduino)**  
- **IN2 (L298N) → Pin 9 (Arduino)**  
- **IN3 (L298N) → Pin 6 (Arduino)**  
- **IN4 (L298N) → Pin 7 (Arduino)**  
- **Motor A terminals → Left Motor**  
- **Motor B terminals → Right Motor**  
- **12V Battery → Motor Driver Power Input**  

### Ultrasonic Sensor (HC-SR04) Connection  
- **VCC → 5V (Arduino)**  
- **GND → GND (Arduino)**  
- **Trig → Pin 3 (Arduino)**  
- **Echo → Pin 4 (Arduino)**  

---

## Procedure  

1. **Connect the Motors and Driver:**  
   - Attach the DC motors to the chassis.  
   - Connect the motors to the motor driver module (L298N).  
   - Wire the motor driver to the Arduino using the control pins.  

2. **Connect the Ultrasonic Sensor:**  
   - Attach the ultrasonic sensor to the front of the robot.  
   - Connect the sensor’s VCC, GND, Trig, and Echo pins to the Arduino.  

3. **Power Connections:**  
   - Connect the battery pack to the motor driver and Arduino.  
   - Add a switch to control power supply.  

4. **Upload the Program:**  
   - Write and upload the Arduino program to enable obstacle detection and movement.  

5. **Test the Movement:**  
   - Ensure the robot moves forward when no obstacles are present.  
   - Test obstacle detection and verify stopping and turning behavior.  

---

## Program Code  

```cpp
const int in_1 = 8;   // Motor driver input pin 1
const int in_2 = 9;   // Motor driver input pin 2
const int in_3 = 6;   // Motor driver input pin 3
const int in_4 = 7;   // Motor driver input pin 4

const int trigPin = 3; // Ultrasonic sensor trigger pin
const int echoPin = 4; // Ultrasonic sensor echo pin

long duration;  // Variable to store the duration of the ultrasonic pulse
float distance; // Variable to store the calculated distance

void setup(){
  pinMode(in_1, OUTPUT);
  pinMode(in_2, OUTPUT);
  pinMode(in_3, OUTPUT);
  pinMode(in_4, OUTPUT);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
}

void loop(){
  // Send a trigger pulse to the ultrasonic sensor
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // Measure the echo duration
  duration = pulseIn(echoPin, HIGH);
  
  // Convert duration to distance (in cm)
  distance = (duration * 0.0343) / 2;

  if (distance <= 30){ 
      // Stop Motors
      digitalWrite(in_1, LOW);
      digitalWrite(in_2, LOW);
      digitalWrite(in_3, LOW);
      digitalWrite(in_4, LOW);
      delay(1000);

      // Turn Right
      digitalWrite(in_1, HIGH);
      digitalWrite(in_2, LOW);
      digitalWrite(in_3, LOW);
      digitalWrite(in_4, HIGH);
      delay(750);
  }
  else {
      // Move Forward
      digitalWrite(in_1, HIGH);
      digitalWrite(in_2, LOW);
      digitalWrite(in_3, HIGH);
      digitalWrite(in_4, LOW);
  }
}
```

---

## Observations  
- The robot moves forward when no obstacles are present.  
- When an obstacle is detected within 30 cm, the robot stops and turns right.  
- After turning, the robot resumes forward movement.  

---

## Result  
The robot successfully detects and avoids obstacles using an ultrasonic sensor and continues navigating towards its destination.  

---

## Precautions  
- Ensure proper wiring to avoid short circuits.  
- Keep the battery connections secure to prevent voltage fluctuations.  
- Calibrate the ultrasonic sensor properly for accurate distance measurement.  
- Test the robot in a controlled environment to avoid damage.  

---

## Conclusion  
The obstacle avoidance robot was successfully implemented using an ultrasonic sensor for real-time obstacle detection and avoidance. The robot demonstrated efficient navigation in a controlled environment.