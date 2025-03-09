# **Publisher-Subscriber Implementation in ROS2**

## **Objective**
The goal of this experiment is to create two ROS 2 nodes—one as a publisher and another as a subscriber—that communicate using topics. The publisher will send string messages, and the subscriber will receive and display them.

## **Requirements**
- Ubuntu 22.04 / 20.04 with **ROS 2 Humble** installed
- Basic understanding of **ROS 2 concepts** (nodes, topics, messages)
- Python 3.x
- `rclpy` and `std_msgs` ROS 2 packages
- `colcon` build tool for ROS 2

## **Theory**
### **What is ROS 2?**
ROS 2 (Robot Operating System 2) is an open-source framework designed for developing robotic applications. It provides tools and libraries to handle complex robotic operations such as message passing, hardware abstraction, and communication between nodes.

### **What are Publisher and Subscriber Nodes?**
- **Publisher Node**: Generates and sends messages over a specific topic.
- **Subscriber Node**: Listens to the same topic and processes the received messages.

In this experiment:
- A **publisher node** (`talker`) will publish "Hello World" messages with an increasing counter.
- A **subscriber node** (`listener`) will receive and display these messages.

---

## **Procedure**

### **1. Create a ROS 2 Package**
Open a terminal, source ROS 2, and create a package named `py_pubsub`:

```bash
source /opt/ros/humble/setup.bash
ros2 pkg create --build-type ament_python py_pubsub
```

### **2. Write the Publisher Node**
Navigate into the package directory:

```bash
cd ros2_ws/src/py_pubsub/py_pubsub
ls
```

Download the publisher script:

```bash
wget https://raw.githubusercontent.com/ros2/examples/foxy/rclpy/topics/minimal_publisher/examples_rclpy_minimal_publisher/publisher_member_function.py
```

#### **Publisher Code (`publisher_member_function.py`)**
```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

---

### **3. Add Dependencies**
Navigate back to the package directory:

```bash
cd ros2_ws/src/py_pubsub
```

Edit `package.xml` and add:

```xml
<exec_depend>rclpy</exec_depend>
<exec_depend>std_msgs</exec_depend>
```

---

### **4. Add an Entry Point**
Modify `setup.py` to register the publisher node:

```python
entry_points={
    'console_scripts': [
        'talker = py_pubsub.publisher_member_function:main',
    ],
},
```

---

### **5. Write the Subscriber Node**
Navigate back to the package directory and download the subscriber script:

```bash
wget https://raw.githubusercontent.com/ros2/examples/foxy/rclpy/topics/minimal_subscriber/examples_rclpy_minimal_subscriber/subscriber_member_function.py
```

#### **Subscriber Code (`subscriber_member_function.py`)**
```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)

def main(args=None):
    rclpy.init(args=args)
    minimal_subscriber = MinimalSubscriber()
    rclpy.spin(minimal_subscriber)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

---

### **6. Add an Entry Point for the Subscriber**
Modify `setup.py` to include the subscriber node:

```python
entry_points={
    'console_scripts': [
        'talker = py_pubsub.publisher_member_function:main',
        'listener = py_pubsub.subscriber_member_function:main',
    ],
},
```

---

### **7. Build and Run the Nodes**
Run the following commands to check dependencies, build the package, and source the setup file:

```bash
rosdep install -i --from-path src --rosdistro humble -y
colcon build --packages-select py_pubsub
source install/setup.bash
```

#### **Run the Publisher**
```bash
ros2 run py_pubsub talker
```
_This should start publishing messages every 0.5 seconds._

#### **Run the Subscriber (in a new terminal)**
```bash
source install/setup.bash
ros2 run py_pubsub listener
```
_The subscriber will start printing received messages._

---

## **Expected Output**
When the publisher is running:
```
[INFO] [minimal_publisher]: Publishing: "Hello World: 0"
[INFO] [minimal_publisher]: Publishing: "Hello World: 1"
[INFO] [minimal_publisher]: Publishing: "Hello World: 2"
...
```
When the subscriber is running:
```
[INFO] [minimal_subscriber]: I heard: "Hello World: 0"
[INFO] [minimal_subscriber]: I heard: "Hello World: 1"
[INFO] [minimal_subscriber]: I heard: "Hello World: 2"
...
```

---

## **Conclusion**
You have successfully implemented a basic Publisher-Subscriber system using ROS 2 in Python. The **publisher node** sends messages, and the **subscriber node** listens and processes them in real-time.

---

## **Additional Notes**
- Use `Ctrl+C` to stop the nodes.
- If changes are made, rebuild using:
  ```bash
  colcon build --packages-select py_pubsub
  ```
- You can change the **topic name** from `"topic"` to any custom name to create multiple publishers and subscribers.
