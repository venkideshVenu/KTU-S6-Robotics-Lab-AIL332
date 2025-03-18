
# **Read Messages from a Bag File (Python/C++) using ROS 2**

## **Aim**
To record and read messages from a ROS 2 bag file, understanding how to store topic data and replay it later for testing and debugging.

---

## **Objective**
- To understand how ROS 2 bag files work for recording and replaying topic messages.
- To use the `ros2 bag` command-line tool to record and play back topic data.
- To explore how to read messages programmatically using Python (optional).

---

## **Theory**
ROS 2 bag files are used for **recording and replaying messages** on topics, similar to a black box for capturing system behavior. The `ros2 bag` tool allows users to save messages from one or more topics into a file and then replay them later. This is useful for debugging, system testing, and sharing data with others.

The process involves **recording messages**, **playing them back**, and optionally **analyzing stored data**. The `ros2 bag record` command captures messages on a given topic, storing them in a `.db3` file. The `ros2 bag play` command replays these messages as if they were being published in real time. Additionally, `ros2 topic echo` can be used to verify that messages are being received correctly.

A common misconception is that a **custom Python script is required to record and replay data**. However, **this is not necessary** since ROS 2 provides built-in commands for managing bag files. The Python script is only useful for advanced cases where **custom message processing** is needed.

---

## **Procedure**

### **Step 1: Create a ROS 2 Package (Optional for Python Usage)**
> **Note:** This step is **not required** for basic recording and playback. However, if you want to **process bag file data programmatically**, follow these steps.

```bash
ros2 pkg create rosbag_py --build-type ament_python --dependencies rclpy rosbag2_py std_msgs
```

Navigate to the package directory:

```bash
cd rosbag_py/rosbag_py
```

Create a Python script:

```bash
gedit record_play.py
```

---

### **Step 2: (Optional) Write a Python Program to Read Bag Data**
**This script is only required if you want to read messages programmatically.**

```python
import rclpy
from rclpy.node import Node
import rosbag2_py

class BagReader(Node):
    def __init__(self):
        super().__init__('bag_reader')
        storage_options = rosbag2_py.StorageOptions(uri='my_rosbag', storage_id='sqlite3')
        converter_options = rosbag2_py.ConverterOptions('', '')
        reader = rosbag2_py.SequentialReader()
        reader.open(storage_options, converter_options)
        
        topic_types = reader.get_all_topics_and_types()
        self.get_logger().info(f"Topics in the bag: {topic_types}")

        while reader.has_next():
            (topic, data, t) = reader.read_next()
            self.get_logger().info(f"Read message from {topic}: {data}")

def main(args=None):
    rclpy.init(args=args)
    node = BagReader()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

---

### **Step 3: Modify `setup.py` to Include the Script (Optional for Python Usage)**
> **Skip this step if you are only using `ros2 bag record` and `ros2 bag play`.**

```bash
gedit setup.py
```

Add the following entry point:

```python
'console_scripts': [
    'record_play = rosbag_py.record_play:main',
],
```

Then, navigate back and build the package:

```bash
cd ..
colcon build --packages-select rosbag_py
source install/setup.bash
```

---

### **Step 4: Record a ROS 2 Topic**
Open a new terminal, source ROS 2, and start recording messages on a topic:

```bash
ros2 bag record -o my_rosbag /example_topic
```

---

### **Step 5: Publish Messages to the Topic**
Open another terminal and publish messages:

```bash
source /opt/ros/humble/setup.bash
ros2 topic pub /example_topic std_msgs/msg/String '{data: "Hello, ROS2!"}' --rate 1
```

Let it run for a while, then **stop the recording** by pressing **Ctrl+C** in the first terminal.

---

### **Step 6: Verify the Recorded Bag File**
Check the bag file details:

```bash
ros2 bag info my_rosbag
```

---

### **Step 7: Replay the Recorded Messages**
In the first terminal, replay the recorded messages:

```bash
ros2 bag play my_rosbag
```

---

### **Step 8: Echo the Topic in a New Terminal**
Check if the messages are being received:

```bash
ros2 topic echo /example_topic
```

---

### **Sample Output:**
```
data: Hello, ROS2!
---
data: Hello, ROS2!
---
data: Hello, ROS2!
---
data: Hello, ROS2!
---
```

---

## **Conclusion**
This experiment demonstrated **how to record and replay messages** using ROS 2 bag files. We used `ros2 bag record` to capture messages, `ros2 bag play` to replay them, and `ros2 topic echo` to verify them.

A custom Python program is **not required** for recording and replaying messages but can be useful if you need **to analyze or process data programmatically**.
