
# **Record and Playback Data using ros2 bag in ROS 2 (Humble)**

## **Aim**
To record and replay topic data in ROS 2 using the `ros2 bag` tool, enabling data logging, debugging, and analysis of robotic behavior.

---

## **Objective**
This experiment aims to demonstrate the use of `ros2 bag` for recording and replaying topic data in ROS 2. By recording movement commands from the **Turtlesim** teleoperation node, we can store this data in a **rosbag** file and later replay it to reproduce the same movement. This experiment will help understand how to capture and analyze ROS 2 topic messages for debugging, testing, and data analysis.

---

## **Theory**
The `ros2 bag` command-line tool is used in **ROS 2** to record data from topics and services. It stores this data in a structured format, allowing it to be **played back later** to recreate previous robotic actions or sensor readings. This is especially useful for **testing and debugging**, as well as for sharing experimental results.

When a **ROS 2 system** runs, multiple topics are continuously exchanging information between nodes. The `ros2 bag record` command captures messages from specified topics and stores them in a database. The stored data can be **inspected** using `ros2 bag info` and **replayed** with `ros2 bag play`, enabling robots to reproduce recorded behaviors. In this experiment, we will record **velocity commands** from the `/turtle1/cmd_vel` topic and replay them to move the **Turtlesim** turtle in the same way as the original teleoperation session.

---

## **Procedure**

### **1. Set Up the ROS 2 Environment**
Open a terminal and source ROS 2:

```bash
source /opt/ros/humble/setup.bash
```

Start the **Turtlesim node**:

```bash
ros2 run turtlesim turtlesim_node
```

Open another terminal and start the **teleoperation node**:

```bash
ros2 run turtlesim turtle_teleop_key
```

---

### **2. Create a Directory for Recording Files**
To keep recordings organized, create a new directory:

```bash
mkdir bag_files
cd bag_files
```

---

### **3. Identify the Topic to Record**
To see the available topics, open a new terminal and run:

```bash
ros2 topic list
```

This will return a list of topics:

```
/parameter_events
/rosout
/turtle1/cmd_vel
/turtle1/color_sensor
/turtle1/pose
```

Since the **teleop node publishes movement commands** to `/turtle1/cmd_vel`, we choose this topic for recording.

To inspect the topic’s published messages:

```bash
ros2 topic echo /turtle1/cmd_vel
```

Press arrow keys in the **teleop terminal** to move the turtle. The terminal running `ros2 topic echo` will display output like:

```yaml
linear:
  x: 2.0
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0
---
```

---

### **4. Record Topic Data**
Move to the `bag_files` directory before recording:

```bash
cd ~/bag_files
```

Start recording the topic:

```bash
ros2 bag record /turtle1/cmd_vel
```

The terminal will display:

```
[INFO] [rosbag2_storage]: Opened database 'rosbag2_YYYY_MM_DD-HH_MM_SS'.
[INFO] [rosbag2_transport]: Listening for topics...
[INFO] [rosbag2_transport]: Subscribed to topic '/turtle1/cmd_vel'
```

Now, **move the turtle** using the teleop terminal. The **commands will be recorded**.

To **stop recording**, press **`Ctrl+C`**.

A new directory `rosbag2_YYYY_MM_DD-HH_MM_SS` will be created containing:
- `metadata.yaml` (file with bag details).
- The recorded data in `.mcap` format.

---

### **5. Inspect the Recorded Data**
To see details of the recorded bag file, use:

```bash
ros2 bag info <bag_file_name>
```

Example:

```bash
ros2 bag info rosbag2_YYYY_MM_DD-HH_MM_SS
```

Output:

```
Files:             subset.mcap
Bag size:          228.5 KiB
Storage id:        mcap
Duration:          48.47s
Start:             Oct 11 2019 06:09:09.12
End                Oct 11 2019 06:09:57.60
Messages:          3013
Topic information: 
  Topic: /turtle1/cmd_vel | Type: geometry_msgs/msg/Twist | Count: 9 | Format: cdr
  Topic: /turtle1/pose    | Type: turtlesim_msgs/msg/Pose | Count: 3004 | Format: cdr
```

---

### **6. Replay the Recorded Data**
Before replaying:
1. **Stop the teleop node** by pressing `Ctrl+C` in its terminal.
2. **Ensure the Turtlesim window is visible**.

To replay the recorded movement:

```bash
ros2 bag play <bag_file_name>
```

Example:

```bash
ros2 bag play rosbag2_YYYY_MM_DD-HH_MM_SS
```

The turtle in the **Turtlesim window will move exactly as it did during recording**.

---

## **Expected Output**
### **Turtlesim Window**
- The turtle **moves in the same pattern** as recorded.

### **Terminal Output (while replaying)**
```
[INFO] [rosbag2_transport]: Replaying messages from rosbag2_YYYY_MM_DD-HH_MM_SS...
[INFO] [rosbag2_transport]: Publishing messages to /turtle1/cmd_vel...
```

---

## **Conclusion**
By completing this experiment, we have successfully demonstrated how to:
✅ **Record** movement data from `/turtle1/cmd_vel`.  
✅ **Store** the data in a `rosbag` file.  
✅ **Replay** the movement using `ros2 bag play`.  

This method is useful for **reproducing robot behavior, debugging, and sharing experimental data**.

---

## **Additional Notes**
- To record **multiple topics**, use:

  ```bash
  ros2 bag record /topic1 /topic2
  ```

- To **list available bag files**, use:

  ```bash
  ls bag_files/
  ```

- If you encounter permission issues, run:

  ```bash
  chmod +x bag_files/*
  ```
