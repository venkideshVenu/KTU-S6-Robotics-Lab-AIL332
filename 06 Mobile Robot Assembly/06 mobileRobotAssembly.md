# Mobile Robot Assembly

## Aim
To design, build, and integrate mechanical, electronic, and software components to create a fully functional autonomous or semi-autonomous robot.

## Objectives
- Assemble the chassis and integrate motors and wheels.
- Connect the motor driver and microcontroller.
- Implement basic movement controls using Arduino.

## Components Required
- Chassis
- Motor
- Motor driver
- Wheels
- Wires
- Arduino Uno board
- Breadboard
- Battery pack
- Switch

## Theory
Mobile robots are robotic systems capable of moving in an environment. They use a combination of mechanical components, electronics, and software to navigate and perform tasks. The integration of motor drivers and microcontrollers allows precise control over movement and direction.


## Procedure
1. **Chassis Preparation**: Attach wheels and motors to the chassis. Secure components with screws or adhesive mounts.
2. **Connect Motors**: Wire the motors to the motor driver module. Connect the motor driver to the microcontroller for control signals.
3. **Power Setup**: Connect the battery pack to the power input of the microcontroller and motor driver. Add a switch to control power.
4. **Upload Program**: Upload the motor control program to the microcontroller. Test forward movement, turning, and stopping.
5. **Test in Controlled Environment**: Run the robot in a controlled environment to evaluate its movement.

## Program Code
```cpp
const int in_1 = 8;   // Left-side motor control pin 1
const int in_2 = 9;   // Left-side motor control pin 2
const int in_3 = 6;   // Right-side motor control pin 3
const int in_4 = 7;   // Right-side motor control pin 4

void setup() {
  pinMode(in_1, OUTPUT);
  pinMode(in_2, OUTPUT);
  pinMode(in_3, OUTPUT);
  pinMode(in_4, OUTPUT);
}

void move(int motor1, int motor2, int motor3, int motor4, int duration) {
  digitalWrite(in_1, motor1);
  digitalWrite(in_2, motor2);
  digitalWrite(in_3, motor3);
  digitalWrite(in_4, motor4);
  delay(duration);
  stopMotors();
}

void stopMotors() {
  digitalWrite(in_1, LOW);
  digitalWrite(in_2, LOW);
  digitalWrite(in_3, LOW);
  digitalWrite(in_4, LOW);
  delay(1000); // Stop for 1 second
}

void loop() {
  move(HIGH, LOW, HIGH, LOW, 3000); // Move forward
  move(LOW, HIGH, LOW, HIGH, 3000); // Move backward
  move(LOW, LOW, HIGH, LOW, 2000);  // Turn left
  move(HIGH, LOW, LOW, LOW, 2000);  // Turn right
}
```

## Observations
- The robot moves forward, backward, left, and right as expected.
- Motor control responds to programmed commands.
- The switch allows manual control of power.

## Result
A functional mobile robot was successfully assembled and programmed for movement.

## Precautions
- Ensure all connections are secure before powering on.
- Use insulated wires to prevent short circuits.
- Test in a safe, open environment to avoid damage.

## Conclusion
The experiment successfully demonstrated the assembly and programming of a mobile robot. The integration of hardware and software allowed precise movement control, achieving the experiment's objectives.
