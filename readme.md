# Robotics Laboratory (AIL332)
![Robotics Lab](https://img.shields.io/badge/Lab-Robotics-blue)
![Academic](https://img.shields.io/badge/Academic-BTech-green)
![Credits](https://img.shields.io/badge/Credits-2-yellow)

This repository contains the implementation and documentation of experiments performed as part of the Robotics Laboratory course (AIL332).

## Course Information
- **Course Code**: AIL332
- **Department**: Computer Science and Engineering (Artificial Intelligence)
- **Category**: PCC
- **Credits**: 2 (0-0-3)
- **Year of Introduction**: 2022

## Course Outcomes
After completion of the course, students will be able to:
1. Interface different peripherals to arduino board
2. Assemble a mobile robot with different sensors and actuators
3. Familiarize about localisation of mobile robots
4. Impart intelligence to robot using standard algorithms
5. Familiarize the robot navigation

## Prerequisites
- Basic knowledge of robotics principles
- Programming experience in Python/C++
- Understanding of Arduino programming
- Familiarity with ROS (Robot Operating System)

## ROS Essentials
- Installing and Configuring ROS Environment (Kinetic/Melodic/Compatible versions)
- ROS fundamentals (Master, nodes, topics, messages, services, parameters and actions)
- ROS Tools – Gazebo, Moveit, Rviz
- Creating Workspace and Package in ROS

## List of Experiments

### Part A: Interfacing sensors and actuators
#### 1. Arduino Basics
   - [LED Interfacing](./01%20Familiarisation%20of%20Arduino%20IDE,%20microcontroller%20&%20IO%20interfacing/1b%20Interfacing%20Arduino%20with%20LED.md)
   - [LCD Interfacing](./01%20Familiarisation%20of%20Arduino%20IDE,%20microcontroller%20&%20IO%20interfacing/1d%20Interfacing%20Arduino%20with%20LCD.md)
   - [Serial Monitor Communication](./01%20Familiarisation%20of%20Arduino%20IDE,%20microcontroller%20&%20IO%20interfacing/1c%20Interfacing%20Arduino%20with%20Serial%20Monitor.md)

#### 2. Sensor Interfacing
   - [IR Sensor](./02%20Interfacing%20IR%20and%20Ultrasonic%20sensor%20with%20Arduino/2a%20Interfacing%20IR%20Sensor.md)
   - [Ultrasonic Sensor](./02%20Interfacing%20IR%20and%20Ultrasonic%20sensor%20with%20Arduino/2b%20Interfacing%20UltraSonic%20Sensor.md)

#### 4. Servo Motor Control
   - [Angle of Rotation](./04%20Interfacing%20Servo%20Motors/04%20Interfacing%20Servo%20Motor.md)

#### 5. Sensor Calibration
   - Sonar Calibration
   - [IR Sensor Calibration](./05%20Calibration%20of%20Sensors/01%20Calibration%20of%20IR%20Sensor.md)
   - Calibration Curves

```{toggle}
Coming Soon !!!







#### 6. [Mobile Robot Assembly](./Part-A/exp6/)

#### 7. [Arduino Networking](./Part-A/exp7/)
   - GSM Integration
   - Bluetooth Communication

### Part B: Intelligent systems
#### 8. [ROS Programming Basics](./Part-B/exp8/)
   - Publisher-Subscriber Implementation
   - Service-Client Programming
   - Data Recording and Playback
   - Bag File Operations

#### 9. [Mobile Robot Localization](./Part-B/exp9/)
   - LIDAR-based Localization
   - ROS Implementation

#### 10. [Touch Sensing](./Part-B/exp10/)
    - Sensor Interfacing
    - Feedback System Implementation

#### 11. [Line Following Robot](./Part-B/exp11/)
    - IR Sensor Implementation
    - Control Algorithm

#### 12. [Obstacle Avoidance](./Part-B/exp12/)
    - Point-to-Point Navigation
    - Obstacle Detection and Avoidance

#### 13. [Object Detection](./Part-B/exp13/)
    - Algorithm Implementation
    - Testing and Validation

#### 14. [ROS Navigation](./Part-B/exp14/)
    - Turtlebot Simulation
    - Navigation Stack Implementation

```

### Additional Programs
   - [Push Button](./00%20Additional%20Programs/01%20Interfacing%20Push%20Button.md)


## How to Use
1. Clone the repository:
   ```bash
   git clone https://github.com/venkideshVenu/KTU-S6-Robotics-Lab-AIL332
   ```

2. Install prerequisites:
   - Arduino IDE
   - ROS (Kinetic/Melodic/Compatible versions)
   - Required Python/C++ packages
3. Navigate to specific experiment directory
4. Follow instructions in individual experiment READMEs

## Tools and Technologies Used
- Arduino IDE and Arduino Board
- ROS (Robot Operating System)
- Python/C++
- Gazebo, Moveit, Rviz
- Various sensors (IR, Ultrasonic, LIDAR, Touch)
- Motors (DC, Servo)
- Turtlebot platform

## Assessment Pattern
- Continuous Internal Evaluation (CIE): 75 marks
- End Semester Examination (ESE): 75 marks
- Duration: 2.5 hours

## References
1. Siegwart, Roland, "Introduction to Autonomous Mobile Robots"
2. Peter Corke, "Robotics, Vision and Control: Fundamental Algorithms in MATLAB"
3. John. J. Craig, "Introduction to Robotics (Mechanics and control)"
4. S K Saha, "Introduction to Robotics"
5. R K Mittal and I J Nagrath, "Robotics and Control"
6. Dahiya, Ravinder S., Valle, Maurizio, "Robotic Tactile Sensing"
7. TurtleBot3 e-Manual: https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation

## License
This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.