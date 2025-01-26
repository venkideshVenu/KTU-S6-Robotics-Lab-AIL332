
# Calibration of IR Sensors and Obtaining the Calibration Curve

---

## Aim
To calibrate an IR sensor and derive the calibration curve to measure distances accurately using the IR sensor.

---

## Components Required
- Arduino Board (e.g., Arduino Uno)
- IR Sensor
- Breadboard
- Jumper Wires
- Serial Monitor (via Arduino IDE)
- A ruler or scale
- Objects to measure distance (flat surface recommended)

---

## Theory
Infrared (IR) sensors detect objects by emitting IR light and measuring the reflected light intensity. The `analogRead()` function on Arduino provides raw sensor values that can be used to compute distances. To establish a relationship between raw values and actual distances, calibration is performed using regression analysis, typically with a linear equation.

---

## Procedure

### **Step 1: Setup Arduino to Record Values**
Use the following Arduino code to measure raw values from the IR sensor for different distances.

#### Arduino Code:
```cpp
#define IR_PIN A0

void setup() {
  Serial.begin(9600);
  Serial.println("Distance (cm), Analog Value");
}

void loop() {
  int sensorValue = analogRead(IR_PIN);
  Serial.println(sensorValue); // Replace with manual obstacle placement
  delay(1000); // Record data at each distance manually
}
```

---

### **Step 2: Record Data**
- Place an object at various distances (e.g., 10 cm, 20 cm, ..., 50 cm) from the sensor.
- Note the corresponding `analogRead()` values for each distance.

---

### **Step 3: Calculate 1/Measured Value**
- Compute the inverse of the measured values (`1/measured_value`).
- Use regression analysis to fit the data to a linear equation.

---

### **Step 4: Perform Regression Analysis**
Fit the data to the equation:  
\[ \text{Distance (cm)} = m \cdot \left(\frac{1}{\text{Measured Value}}\right) + b \]  
Where:
- \( m \): Slope
- \( b \): Intercept

#### Python Example for Regression:
```python
import numpy as np
from sklearn.linear_model import LinearRegression

# Data
distances = np.array([10, 20, 30, 40, 50]).reshape(-1, 1)  # cm
measured_values = np.array([600, 400, 300, 250, 200])  # analogRead() values
inverted_values = 1 / measured_values  # 1/Measured Value

# Linear regression
model = LinearRegression().fit(inverted_values.reshape(-1, 1), distances)
m = model.coef_[0]  # Slope
b = model.intercept_  # Intercept

print(f"Calibration equation: Distance = {m:.4f} * (1/Analog Value) + {b:.4f}")
```

#### Example Output:
\[
\text{Calibration Equation: Distance (cm)} = 7000 \cdot \left(\frac{1}{\text{Measured Value}}\right) - 5
\]

---

### **Step 5: Update the Arduino Code**
Use the calibration equation to measure distances dynamically with the IR sensor.

#### Updated Arduino Code:
```cpp
#define IR_PIN A0

void setup() {
  Serial.begin(9600);
}

void loop() {
  int sensorValue = analogRead(IR_PIN);
  float inverseValue = 1.0 / sensorValue; // Calculate 1/Measured Value

  // Calibration equation: Distance = m * (1/Measured Value) + b
  float m = 7000; // Replace with your calculated slope
  float b = -5;   // Replace with your calculated intercept
  float distance = m * inverseValue + b;

  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.println(" cm");
  delay(500);
}
```

---

### **Step 6: Verification**
1. Place objects at known distances from the sensor.
2. Compare the measured distance (calculated using the calibration equation) with the actual distance.
3. Verify the accuracy of the sensor in various conditions (e.g., ambient light).

---

## Notes:
- Ensure proper environmental conditions for accurate calibration:
  - Avoid strong ambient light interference.
  - Use a consistent surface for the object being measured.
- If the linear model does not fit well, consider using a polynomial regression or another regression method.
- Regression tools like Python, Excel, or MATLAB can be used for calculations.

---

## Troubleshooting
1. **No Output on Serial Monitor**:
   - Check the connection between the IR sensor and the Arduino.
   - Verify the correct pin number is used for the IR sensor in the code.
   
2. **Inconsistent Data**:
   - Ensure the sensor and object are stable during measurements.
   - Avoid reflective or uneven surfaces for the object.

3. **Incorrect Distance Calculation**:
   - Double-check the regression parameters (\( m \) and \( b \)) in the Arduino code.
   - Recalibrate if necessary.

---

## Conclusion
By calibrating the IR sensor and deriving the calibration curve, we can accurately measure distances using the sensor and the Arduino board. The calibration equation provides a reliable way to convert raw sensor values into meaningful distance measurements.