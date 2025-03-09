# **Service-Client Implementation in ROS2 (Python)**

## **Objective**
The objective of this experiment is to implement a **service-client** communication model in ROS 2 using Python. The example used here is a simple integer addition system where:
- The **service node** provides a function to add two integers.
- The **client node** requests the sum of two integers and receives the result.

---

## **Requirements**
- Ubuntu 22.04 / 20.04 with **ROS 2 Humble** installed
- Basic understanding of **ROS 2 concepts** (nodes, services, requests, responses)
- Python 3.x
- `rclpy` and `example_interfaces` ROS 2 packages
- `colcon` build tool for ROS 2

---

## **Theory**
### **What are Services and Clients in ROS 2?**
- A **Service** in ROS 2 follows a request-response model.
- A **Client** sends a request to a service and waits for a response.
- Unlike **publishers and subscribers**, where communication is continuous, **services and clients** operate only when needed.

In this experiment:
- The **service node** (`service_member_function.py`) waits for integer addition requests.
- The **client node** (`client_member_function.py`) sends two integers to the service and receives the sum.

---

## **Procedure**

### **1. Create a ROS 2 Package**
Open a terminal, source ROS 2, and create a package named `py_srvcli`:

```bash
source /opt/ros/humble/setup.bash
ros2 pkg create --build-type ament_python py_srvcli --dependencies rclpy example_interfaces
```

This command will generate a new package **py_srvcli** with all necessary files.

---

### **2. Update package.xml**
Since we used the `--dependencies` option, `rclpy` and `example_interfaces` are automatically added. However, ensure that the `package.xml` file contains:

```xml
<description>Python client-server tutorial</description>
<maintainer email="you@email.com">Your Name</maintainer>
<license>Apache License 2.0</license>
```

---

### **3. Update setup.py**
Modify `setup.py` to include package details:

```python
maintainer='Your Name',
maintainer_email='you@email.com',
description='Python client-server tutorial',
license='Apache License 2.0',
```

---

### **4. Write the Service Node**
Inside the `ros2_ws/src/py_srvcli/py_srvcli/` directory, create a file called **`service_member_function.py`** and add the following code:

#### **Service Node (`service_member_function.py`)**
```python
from example_interfaces.srv import AddTwoInts
import rclpy
from rclpy.node import Node

class MinimalService(Node):
    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info(f'Incoming request\na: {request.a} b: {request.b}')
        return response

def main(args=None):
    rclpy.init(args=args)
    minimal_service = MinimalService()
    rclpy.spin(minimal_service)
    minimal_service.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

This **MinimalService** node:
- Creates a service named **`add_two_ints`**.
- Listens for requests and computes the sum of two integers.

---

### **5. Write the Client Node**
Inside the `ros2_ws/src/py_srvcli/py_srvcli/` directory, create a file called **`client_member_function.py`** and add the following code:

#### **Client Node (`client_member_function.py`)**
```python
import sys
from example_interfaces.srv import AddTwoInts
import rclpy
from rclpy.node import Node

class MinimalClientAsync(Node):
    def __init__(self):
        super().__init__('minimal_client_async')
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        self.req = AddTwoInts.Request()

    def send_request(self, a, b):
        self.req.a = a
        self.req.b = b
        return self.cli.call_async(self.req)

def main():
    rclpy.init()
    minimal_client = MinimalClientAsync()
    future = minimal_client.send_request(int(sys.argv[1]), int(sys.argv[2]))
    rclpy.spin_until_future_complete(minimal_client, future)
    response = future.result()
    minimal_client.get_logger().info(f'Result of add_two_ints: {sys.argv[1]} + {sys.argv[2]} = {response.sum}')
    minimal_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

This **MinimalClientAsync** node:
- Waits for the service **`add_two_ints`** to be available.
- Sends two integers as a request.
- Receives and prints the sum.

---

### **6. Add an Entry Point**
Modify the `setup.py` file to include the **service** and **client** entry points:

```python
entry_points={
    'console_scripts': [
        'service = py_srvcli.service_member_function:main',
        'client = py_srvcli.client_member_function:main',
    ],
},
```

---

### **7. Build and Run the Nodes**
#### **Check Dependencies**
Before building, ensure dependencies are installed:

```bash
rosdep install -i --from-path src --rosdistro humble -y
```

#### **Build the Package**
Navigate to the ROS 2 workspace root and build the package:

```bash
colcon build --packages-select py_srvcli
```

#### **Source the Setup File**
```bash
source install/setup.bash
```

---

### **8. Run the Service and Client**
#### **Start the Service Node**
Open a terminal and run:

```bash
ros2 run py_srvcli service
```

_This will start the service, which will wait for client requests._

#### **Start the Client Node**
Open another terminal, source ROS 2, and run:

```bash
ros2 run py_srvcli client 2 3
```

_This will send a request to add **2 + 3**, and the service will return the result._

---

## **Expected Output**
### **Service Node Output**
```
[INFO] [minimal_service]: Incoming request
a: 2 b: 3
```

### **Client Node Output**
```
[INFO] [minimal_client_async]: Result of add_two_ints: 2 + 3 = 5
```

---

## **Conclusion**
You have successfully implemented a **Service-Client communication system** in ROS 2. The **service node** receives a request, processes it, and responds to the **client node** with the computed sum.

---

## **Additional Notes**
- Use `Ctrl+C` to stop the nodes.
- If you make changes, rebuild using:
  ```bash
  colcon build --packages-select py_srvcli
  ```
- You can modify the **service function** to perform different computations.

