# Experiment: Line Following Robot  

## Aim  
To design and implement a line-following robot that uses infrared (IR) sensors to detect and track a predefined path.  

---

## Components Required  
- **IR Sensors**  
- **Chassis Kit**  
- **Jumper Wires**  
- **Batteries**  
- **Motor Driver (L298N)**  
- **DC Motors**  
- **Arduino Uno Board**  
- **Wheels**  

---

## Circuit Description  

### IR Sensor Connection  
- **VCC (IR Sensor) → 5V (Arduino)**  
- **GND (IR Sensor) → GND (Arduino)**  
- **Left Sensor Output → Pin 13 (Arduino)**  
- **Right Sensor Output → Pin 12 (Arduino)**  

### Motor Driver (L298N) Connection  
- **IN1 (L298N) → Pin 3 (Arduino)**  
- **IN2 (L298N) → Pin 5 (Arduino)**  
- **IN3 (L298N) → Pin 6 (Arduino)**  
- **IN4 (L298N) → Pin 9 (Arduino)**  
- **Motor A Terminals → Left Motor**  
- **Motor B Terminals → Right Motor**  
- **12V Battery → Motor Driver Power Input**  

---

## Procedure  

1. **Assemble the Chassis**  
   - Attach the wheels to the motors.  
   - Securely fix the motors to the chassis.  

2. **Connect the Motor Driver**  
   - Connect the DC motors to the motor driver (L298N).  
   - Wire the control pins (IN1, IN2, IN3, IN4) to the Arduino.  

3. **Position the IR Sensors**  
   - Mount the IR sensors in front of the robot, close to the ground.  
   - Align them to detect the line effectively.  

4. **Connect Power Supply**  
   - Provide power to the motor driver and Arduino using batteries.  

5. **Upload the Code**  
   - Open Arduino IDE and upload the code.  

6. **Test the Robot**  
   - Observe if the robot follows the line, turns left or right, and stops correctly.  

---

## Arduino Code  

```cpp
int mot1 = 9;  // IN4
int mot2 = 6;  // IN3
int mot3 = 5;  // IN2
int mot4 = 3;  // IN1
int left = 13; // Left IR sensor
int right = 12; // Right IR sensor

int Left = 0;
int Right = 0;

void LEFT();
void RIGHT();
void STOP();

void setup() {
  pinMode(mot1, OUTPUT);
  pinMode(mot2, OUTPUT);
  pinMode(mot3, OUTPUT);
  pinMode(mot4, OUTPUT);
  pinMode(left, INPUT);
  pinMode(right, INPUT);

  digitalWrite(left, HIGH);  
  digitalWrite(right, HIGH);
}

void loop() {
  analogWrite(mot1, 255);
  analogWrite(mot2, 0);
  analogWrite(mot3, 255);
  analogWrite(mot4, 0);

  while (1) {
    Left = digitalRead(left);
    Right = digitalRead(right);

    if (Left == 0 && Right == 1) {
      LEFT();
    } 
    else if (Right == 0 && Left == 1) {
      RIGHT();
    }
  }
}

void LEFT() {
  analogWrite(mot3, 0);
  analogWrite(mot4, 30);

  while (Left == 0) {
    Left = digitalRead(left);
    Right = digitalRead(right);

    if (Right == 0) {
      int lprev = Left;
      int rprev = Right;
      STOP();

      while (lprev == Left && rprev == Right) {
        Left = digitalRead(left);
        Right = digitalRead(right);
      }
    }

    analogWrite(mot1, 255);
    analogWrite(mot2, 0);
  }
  analogWrite(mot3, 255);
  analogWrite(mot4, 0);
}

void RIGHT() {
  analogWrite(mot1, 0);
  analogWrite(mot2, 30);

  while (Right == 0) {
    Left = digitalRead(left);
    Right = digitalRead(right);

    if (Left == 0) {
      int lprev = Left;
      int rprev = Right;
      STOP();

      while (lprev == Left && rprev == Right) {
        Left = digitalRead(left);
        Right = digitalRead(right);
      }
    }

    analogWrite(mot3, 255);
    analogWrite(mot4, 0);
  }
  analogWrite(mot1, 255);
  analogWrite(mot2, 0);
}

void STOP() {
  analogWrite(mot1, 0);
  analogWrite(mot2, 0);
  analogWrite(mot3, 0);
  analogWrite(mot4, 0);
}
```

---

## Observations  
- The robot moves forward when both sensors detect the white surface.  
- The robot turns **left** when the left sensor detects the black line.  
- The robot turns **right** when the right sensor detects the black line.  
- The robot stops if both sensors detect black.  

---

## Result  
The line-following robot was successfully designed and implemented. It was able to detect and follow a black line using IR sensors and move accordingly.  

---

## Precautions  
- Ensure all connections are secure to avoid signal loss.  
- Calibrate the IR sensors for proper line detection.  
- Place the sensors at an optimal height from the surface.  

---

## Conclusion  
The experiment was successfully completed. The line-following robot effectively detected the predefined path and navigated accordingly using IR sensors and motor driver control.  