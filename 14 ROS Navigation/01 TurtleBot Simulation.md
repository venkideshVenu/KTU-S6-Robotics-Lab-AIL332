# **TurtleBot Simulation using ROS 2 (Humble)**

## **Objective**
The objective of this experiment is to simulate the **TurtleBot movement** in ROS 2 and control it to draw a circular path using Python. We will:
- Set up a ROS 2 workspace and package.
- Write a Python script to control **Turtlesim**.
- Execute the node to simulate motion.

---

## **Requirements**
- **Ubuntu 22.04 / 20.04** with **ROS 2 Humble** installed.
- Basic knowledge of **ROS 2 concepts** (nodes, messages, topics).
- **Turtlesim package** (default in ROS 2).

---

## **Theory**
### **What is Turtlesim?**
- `turtlesim` is a built-in ROS 2 package that simulates a simple robot (turtle) moving in a **2D environment**.
- It subscribes to velocity commands (`geometry_msgs/Twist`) on the topic **`/turtle1/cmd_vel`**.

In this experiment:
- We create a **custom ROS 2 package**.
- We write a **Python node** (`my_node.py`) to send velocity commands to the turtle.
- The turtle moves in a **circular path**.

---

## **Procedure**

### **1. Set Up the ROS 2 Environment**
Open a terminal and source ROS 2:

```bash
source /opt/ros/humble/setup.bash
```

Create a new workspace:

```bash
mkdir -p turtle_tutorials/src
cd turtle_tutorials/src
```

---

### **2. Create a New ROS 2 Package**
Run the following command to create a package named `my_package`:

```bash
ros2 pkg create --build-type ament_python --node-name my_node my_package
```

This command will generate:
- A package **`my_package`** inside `turtle_tutorials/src`.
- A default Python node **`my_node.py`**.

Navigate to the package directory:

```bash
cd my_package/my_package
```

---

### **3. Write the Python Node**
Create and edit the node file:

```bash
gedit my_node.py
```

Copy and paste the following code:

#### **Python Node (`my_node.py`)**
```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class MoveTurtle(Node):
    def __init__(self):
        super().__init__('move_turtle')
        self.publisher_ = self.create_publisher(Twist, 'turtle1/cmd_vel', 10)
        timer_period = 0.1  # Timer callback every 0.1 seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.angular_speed = 0.5  # Angular velocity for circular motion
        self.linear_speed = 0.5   # Linear velocity for forward movement

    def timer_callback(self):
        msg = Twist()
        msg.linear.x = self.linear_speed
        msg.angular.z = self.angular_speed
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    move_turtle = MoveTurtle()
    try:
        rclpy.spin(move_turtle)
    except KeyboardInterrupt:
        pass
    move_turtle.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

This script:
- **Publishes velocity commands** to the topic **`/turtle1/cmd_vel`**.
- Moves the turtle in a **circular trajectory** using **linear and angular velocity**.

---

### **4. Make the Node Executable**
Give execute permission:

```bash
chmod +x my_node.py
```

Move back to the ROS 2 workspace:

```bash
cd ../../../
```

---

### **5. Run the Turtlesim Simulation**
#### **Start the Turtlesim Node**
Open a new terminal, source ROS 2, and run:

```bash
source /opt/ros/humble/setup.bash
ros2 run turtlesim turtlesim_node
```

This will open the **Turtlesim simulation window**.

---

### **6. Run the Custom Python Node**
In another terminal, source ROS 2 and navigate to the package directory:

```bash
source /opt/ros/humble/setup.bash
cd turtle_tutorials/src/my_package/my_package
```

Run the Python script:

```bash
python3 my_node.py
```

This will start sending velocity commands, and the turtle will move in a **circular path**.

---

## **Expected Output**
### **Turtlesim Window**
- The turtle moves continuously in a **circular trajectory**.

### **Terminal Output**
```
[INFO] [move_turtle]: Publishing velocity command: linear=0.5, angular=0.5
[INFO] [move_turtle]: Publishing velocity command: linear=0.5, angular=0.5
...
```

---

## **Conclusion**
You have successfully simulated **Turtlesim navigation** in ROS 2. The **Turtlesim node** receives velocity commands from our custom **Python node** and moves in a **circular pattern**.

---

## **Additional Notes**
- To stop the movement, press **`Ctrl+C`** in the terminal running `my_node.py`.
- You can modify `angular_speed` and `linear_speed` in the Python script for different motion patterns.
- If you make changes, **re-run the script**:
  ```bash
  python3 my_node.py
  ```