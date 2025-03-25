# 🤖 Robotics Laboratory (AIL332)

[![Robotics Lab](https://img.shields.io/badge/Lab-Robotics-blue)](https://github.com/venkideshVenu/KTU-S6-Robotics-Lab-AIL332)
[![Academic](https://img.shields.io/badge/Academic-BTech-green)](https://github.com/venkideshVenu/KTU-S6-Robotics-Lab-AIL332)
[![Credits](https://img.shields.io/badge/Credits-2-yellow)](https://github.com/venkideshVenu/KTU-S6-Robotics-Lab-AIL332)
[![Arduino](https://img.shields.io/badge/Arduino-Used-teal)](https://github.com/venkideshVenu/KTU-S6-Robotics-Lab-AIL332)
[![ROS](https://img.shields.io/badge/ROS-Implemented-orange)](https://github.com/venkideshVenu/KTU-S6-Robotics-Lab-AIL332)
[![MIT License](https://img.shields.io/badge/License-MIT-purple)](https://github.com/venkideshVenu/KTU-S6-Robotics-Lab-AIL332)

Welcome to the Robotics Laboratory (AIL332) repository! This comprehensive collection contains implementations and documentation for experiments conducted as part of the BTech Robotics Laboratory course. Designed to provide a hands-on experience with robotics systems, this lab covers everything from basic Arduino interfacing to advanced ROS-based navigation.

<div align="center">
  <img src="https://raw.githubusercontent.com/github/explore/80688e429a7d4ef2fca1e82350fe8e3517d3494d/topics/arduino/arduino.png" width="60px">
  <img src="https://upload.wikimedia.org/wikipedia/commons/b/bb/Ros_logo.svg" width="60px">
  <img src="https://upload.wikimedia.org/wikipedia/en/5/5e/Gazebo_logo_without_text.svg" width="60px">
</div>

## 📚 Course Information

| Detail | Description |
|--------|-------------|
| **Course Code** | AIL332 |
| **Department** | Computer Science and Engineering (Artificial Intelligence) |
| **Category** | PCC (Professional Core Course) |
| **Credits** | 2 (0-0-3) |
| **Year of Introduction** | 2022 |
| **Syllabus** | [Click Here](./Robotics%20Lab%20Syllabus.pdf) |
| **Semester** | S6 (Sixth Semester) |

## 🎯 Course Outcomes

After completing this course, students will be able to:

1. **CO1:** Interface various peripherals to Arduino boards for robotics applications
2. **CO2:** Assemble mobile robots with different sensors and actuators
3. **CO3:** Understand and implement mobile robot localization techniques
4. **CO4:** Apply standard algorithms to enhance robot intelligence and functionality
5. **CO5:** Master robot navigation principles and implement autonomous navigation systems

## 🔧 Prerequisites

- ✅ Basic knowledge of robotics principles and autonomous systems
- ✅ Programming experience in Python and C++
- ✅ Understanding of Arduino programming and microcontroller basics
- ✅ Familiarity with ROS (Robot Operating System) concepts
- ✅ Basic electronics knowledge for sensor and actuator interfacing

## 🌐 ROS Essentials

This laboratory introduces the Robot Operating System (ROS) as a framework for robot software development:

- 🔹 Installing and configuring ROS environment (Kinetic/Melodic/Noetic or compatible versions)
- 🔹 ROS fundamentals: Master, nodes, topics, messages, services, parameters, and actions
- 🔹 ROS Tools: Gazebo (simulation), Moveit (motion planning), Rviz (visualization)
- 🔹 Creating Workspaces and Packages in ROS for modular development
- 🔹 Implementing Publisher-Subscriber and Service-Client communication patterns

## 🧪 Laboratory Experiments

### Part A: Interfacing Sensors and Actuators

#### 1. Arduino Basics
- [LED Interfacing](./01%20Familiarisation%20of%20Arduino%20IDE,%20microcontroller%20&%20IO%20interfacing/1b%20Interfacing%20Arduino%20with%20LED.md) - Control LEDs with Arduino for basic I/O operations
- [LCD Interfacing](./01%20Familiarisation%20of%20Arduino%20IDE,%20microcontroller%20&%20IO%20interfacing/1d%20Interfacing%20Arduino%20with%20LCD.md) - Display information on LCD screens
- [Serial Monitor Communication](./01%20Familiarisation%20of%20Arduino%20IDE,%20microcontroller%20&%20IO%20interfacing/1c%20Interfacing%20Arduino%20with%20Serial%20Monitor.md) - Establish communication between Arduino and computer

#### 2. Sensor Interfacing
- [IR Sensor](./02%20Interfacing%20IR%20and%20Ultrasonic%20sensor%20with%20Arduino/2a%20Interfacing%20IR%20Sensor.md) - Detect obstacles using infrared technology
- [Ultrasonic Sensor](./02%20Interfacing%20IR%20and%20Ultrasonic%20sensor%20with%20Arduino/2b%20Interfacing%20UltraSonic%20Sensor.md) - Measure distances using sound waves

#### 3. DC Motor Control
- [Speed and Direction Control](./03%20Interfacing%20DC%20Motor/03%20Interfacing%20DC%20Motor.md) - Control motor speed and direction for robot movement

#### 4. Servo Motor Control
- [Angle of Rotation](./04%20Interfacing%20Servo%20Motors/04%20Interfacing%20Servo%20Motor.md) - Precise control of servo motors for robotics applications

#### 5. Sensor Calibration
- [IR Sensor Calibration](./05%20Calibration%20of%20Sensors/01%20Calibration%20of%20IR%20Sensor.md) - Calibrate sensors for accurate readings
- Sonar Calibration - Configure ultrasonic sensors for precise distance measurement
- Calibration Curves - Generate and interpret sensor calibration data

#### 6. Mobile Robot Assembly
- [Mobile Robot Assembly](./06%20Mobile%20Robot%20Assembly/06%20mobileRobotAssembly.md) - Build a complete mobile robot with various components

#### 7. Arduino Networking (Coming Soon)
- GSM Integration - Connect robots to cellular networks
- Bluetooth Communication - Implement wireless robot control

### Part B: Intelligent Systems

#### 8. ROS Programming Basics
- [Publisher-Subscriber Implementation](./08%20ROS%20Programming%20Basics/01%20Publisher-Subscriber.md) - Implement basic ROS communication patterns
- [Service-Client Programming](./08%20ROS%20Programming%20Basics/02%20Service%20Client%20Programming.md) - Create request-response based interactions
- [Recording and Playing Back Data](./08%20ROS%20Programming%20Basics/03%20recordAndPlaybackData.md) - Use rosbag for data recording and playback
- [Reading Messages from Bag File](./08%20ROS%20Programming%20Basics/04%20ReadFromBagFile.md) - Extract and process recorded sensor data

#### 9. Mobile Robot Localization (Coming Soon)
- LIDAR-based Localization - Implement laser-based robot positioning
- ROS Implementation - Integrate localization algorithms in ROS

#### 10. Touch Sensing (Coming Soon)
- Sensor Interfacing - Connect touch sensors to robots
- Feedback System Implementation - Create responsive touch-based feedback systems

#### 11. Line Following Robot
- [Line Following Robot](./11%20Line%20Following%20Robot/01%20Line%20Following%20Robot.md) - Build and program a robot that follows a line path

#### 12. Obstacle Avoidance
- [Obstacle Avoidance Robot](./12%20Obstacle%20Avoidance%20Robot/01%20ObstacleAvoidanceRobot.md) - Create a robot that detects and avoids obstacles

#### 13. Object Detection
- [Object Detection using a Standard Algorithm](./13%20ObjectDetection/01%20ObjectDetection.md) - Implement computer vision for object recognition

#### 14. ROS Navigation
- [Turtlebot Simulation](./14%20ROS%20Navigation/01%20TurtleBot%20Simulation.md) - Simulate autonomous navigation using Turtlebot platform

### Additional Programs and Resources

- [Push Button Interfacing](./00%20Additional%20Programs/01%20Interfacing%20Push%20Button.md) - Learn how to use push buttons for robot interaction
- More examples and utilities will be added throughout the course

## 🚀 How to Use This Repository

1. **Clone the repository:**
   ```bash
   git clone https://github.com/venkideshVenu/KTU-S6-Robotics-Lab-AIL332
   cd KTU-S6-Robotics-Lab-AIL332
   ```

2. **Install prerequisites:**
   ```bash
   # Arduino IDE
   sudo apt-get install arduino

   # ROS Installation (Ubuntu 20.04 - ROS Noetic example)
   sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
   sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
   sudo apt-get update
   sudo apt-get install ros-noetic-desktop-full

   # Python packages
   pip install numpy matplotlib pandas
   ```

3. **Navigate to specific experiment directory**
   ```bash
   cd "01 Familiarisation of Arduino IDE, microcontroller & IO interfacing"
   ```

4. **Follow instructions in individual experiment READMEs**

## 🛠️ Tools and Technologies

<table>
  <tr>
    <td align="center"><img src="https://raw.githubusercontent.com/github/explore/80688e429a7d4ef2fca1e82350fe8e3517d3494d/topics/arduino/arduino.png" width="40px" height="40px"><br>Arduino</td>
    <td align="center"><img src="https://upload.wikimedia.org/wikipedia/commons/b/bb/Ros_logo.svg" width="40px" height="40px"><br>ROS</td>
    <td align="center"><img src="https://raw.githubusercontent.com/github/explore/80688e429a7d4ef2fca1e82350fe8e3517d3494d/topics/python/python.png" width="40px" height="40px"><br>Python</td>
    <td align="center"><img src="https://raw.githubusercontent.com/github/explore/80688e429a7d4ef2fca1e82350fe8e3517d3494d/topics/cpp/cpp.png" width="40px" height="40px"><br>C++</td>
  </tr>
  <tr>
    <td align="center"><img src="https://upload.wikimedia.org/wikipedia/en/5/5e/Gazebo_logo_without_text.svg" width="40px" height="40px"><br>Gazebo</td>
    <td align="center">🤖<br>Moveit</td>
    <td align="center">🔍<br>Rviz</td>
    <td align="center">🔌<br>Sensors & Motors</td>
  </tr>
</table>

## 📊 Assessment Pattern

| Component | Marks | Details |
|-----------|-------|---------|
| Continuous Internal Evaluation (CIE) | 75 | Lab performance, records & viva voce |
| End Semester Examination (ESE) | 75 | Practical exam (2.5 hours) |
| **Total** | **150** | |

### Grading Criteria
- **Lab Performance:** 40% - Quality of implementation and demonstration
- **Lab Records:** 30% - Documentation and analysis of experiments
- **Viva Voce:** 30% - Understanding of concepts and applications

## 📚 References

1. Siegwart, Roland, "Introduction to Autonomous Mobile Robots", MIT Press
2. Peter Corke, "Robotics, Vision and Control: Fundamental Algorithms in MATLAB", Springer
3. John. J. Craig, "Introduction to Robotics (Mechanics and control)", Pearson
4. S K Saha, "Introduction to Robotics", McGraw Hill
5. R K Mittal and I J Nagrath, "Robotics and Control", Tata McGraw Hill
6. Dahiya, Ravinder S., Valle, Maurizio, "Robotic Tactile Sensing", Springer
7. [TurtleBot3 e-Manual](https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation)
8. [ROS Wiki](http://wiki.ros.org/)
9. [Arduino Documentation](https://www.arduino.cc/reference/en/)

## 🔄 Contributing

Contributions to improve this repository are welcome! Please follow these steps:

1. Fork the repository
2. Create a feature branch (`git checkout -b feature/amazing-feature`)
3. Commit your changes (`git commit -m 'Add amazing feature'`)
4. Push to the branch (`git push origin feature/amazing-feature`)
5. Open a Pull Request

## 📝 License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

---

<div align="center">
  <p>
    <i>Made with ❤️ for Robotics Students</i>
  </p>
  <p>
    <a href="https://github.com/venkideshVenu/KTU-S6-Robotics-Lab-AIL332/issues">Report Bug</a>
    ·
    <a href="https://github.com/venkideshVenu/KTU-S6-Robotics-Lab-AIL332/issues">Request Feature</a>
    ·
    <a href="https://github.com/venkideshVenu/KTU-S6-Robotics-Lab-AIL332/discussions">Ask Question</a>
  </p>
</div>