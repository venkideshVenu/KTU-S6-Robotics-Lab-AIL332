# Viva Questions – Robotics Lab (Electronics Components)

## 📘 Table of Contents

- [Arduino Uno R3](#arduino-uno-r3)
- [LCD 16x2 I2C](#lcd-16x2-with-i2c-module)
- [Ultrasonic Sensor (HC-SR04)](#ultrasonic-sensor-hc-sr04)
- [L298N Motor Driver](#l298n-motor-driver-module)
- [Servo Motor](#servo-motor)
- [IR Sensor](#ir-sensor)
- [Touch Sensor](#touch-sensor)

---



# Arduino UNO R3
---
## 🔹 **Basic Questions (10 Q&A)**

1. **What is Arduino Uno R3?**  
   *A microcontroller board based on the ATmega328P, used for prototyping and automation projects.*

2. **Who developed Arduino?**  
   *Arduino was initially developed by Massimo Banzi and David Cuartielles in Italy.*

3. **What does 'R3' indicate in Arduino Uno R3?**  
   *Revision 3 – the third and latest revision of the Arduino Uno board.*

4. **What is the microcontroller used in Arduino Uno?**  
   *ATmega328P.*

5. **What is the function of Arduino?**  
   *It reads input from sensors or switches and controls output devices like LEDs, motors, etc., based on code uploaded to it.*

6. **What is the core language used to program Arduino?**  
   *Arduino is programmed using a simplified version of C/C++.*

7. **What is the default operating frequency of Arduino Uno R3?**  
   *16 MHz.*

8. **What is the default logic level of Arduino Uno?**  
   *5V logic level.*

9. **Is Arduino open-source?**  
   *Yes, both hardware and software of Arduino are open-source.*

10. **What is the size and shape of an Arduino Uno board?**  
   *It is a rectangular board roughly 68.6 mm × 53.4 mm.*

---

## 🔹 **Pin and Memory Related (10 Q&A)**

1. **How many digital I/O pins does Arduino Uno have?**  
   *14 digital I/O pins (D0 to D13).*

2. **Which digital pins support PWM?**  
   *Pins 3, 5, 6, 9, 10, and 11.*

3. **How many analog input pins are there?**  
   *6 analog input pins (A0 to A5).*

4. **What is the resolution of analog-to-digital conversion?**  
   *10-bit resolution (1024 levels).*

5. **What is the flash memory size of Arduino Uno?**  
   *32 KB, with 0.5 KB used by the bootloader.*

6. **How much SRAM does Arduino Uno have?**  
   *2 KB.*

7. **How much EEPROM does Arduino Uno have?**  
   *1 KB.*

8. **What is the use of the AREF pin?**  
   *To set a reference voltage for analog inputs.*

9. **Which pin is used to reset the board?**  
   *The RESET pin.*

10. **What happens when the memory is full?**  
   *The program may crash or behave unexpectedly if SRAM or EEPROM is overused.*

---

## 🔹 **Communication (10 Q&A)**

1. **Which communication protocols are supported by Arduino Uno?**  
   *UART (Serial), SPI, and I2C.*

2. **What is UART used for?**  
   *For serial communication with devices like PCs, GPS, GSM modules, etc.*

3. **Which pins are used for Serial Communication?**  
   *Pin 0 (RX) and Pin 1 (TX).*

4. **How many SPI pins are available?**  
   *4: MISO (12), MOSI (11), SCK (13), SS (10).*

5. **Which pins are used for I2C?**  
   *A4 (SDA), A5 (SCL).*

6. **What is the use of the USB connector on Arduino Uno?**  
   *For programming the board and power supply from a computer.*

7. **What is the baud rate of serial communication by default?**  
   *9600 bps (can be changed).*

8. **What does the TX LED indicate?**  
   *Transmitting data via serial.*

9. **What does the RX LED indicate?**  
   *Receiving data via serial.*

10. **Can we use software serial in Arduino Uno?**  
   *Yes, using the SoftwareSerial library to emulate serial ports on other digital pins.*

---

## 🔹 **Practical Use / Application (10 Q&A)**

1. **What software is used to program Arduino?**  
   *Arduino IDE.*

2. **What type of USB cable is used with Arduino Uno?**  
   *USB Type-B cable.*

3. **Can Arduino run without a computer?**  
   *Yes, after uploading the code, it can run independently when powered.*

4. **How can we power Arduino without USB?**  
   *Using DC barrel jack (7–12V) or VIN pin.*

5. **Can Arduino be used to control motors?**  
   *Yes, via motor drivers or shields.*

6. **How do we upload code to Arduino?**  
   *By clicking the upload button on the Arduino IDE.*

7. **Can Arduino be used to read sensor data?**  
   *Yes, using analogRead and digitalRead functions.*

8. **What are shields in Arduino?**  
   *Plug-in modules that add functionality like Wi-Fi, motor control, etc.*

9. **What are some common projects using Arduino Uno?**  
   *Home automation, obstacle-avoiding robot, temperature monitor, etc.*

10. **How do we debug Arduino code?**  
   *Using Serial Monitor to print values or messages.*

---

## 🔹 **Advanced / EC-Related Questions (10 Q&A)**

1. **What is the function of the voltage regulator on the board?**  
   *To provide a constant 5V to the microcontroller.*

2. **What is the use of the 16 MHz crystal oscillator?**  
   *To provide the clock signal required for timing operations.*

3. **What are interrupts?**  
   *Interrupts allow the microcontroller to pause its current task to handle an event immediately.*

4. **Which pins are interrupt capable on Arduino Uno?**  
   *Pins 2 and 3 (INT0 and INT1).*

5. **How is analogWrite different from digitalWrite?**  
   *analogWrite outputs a PWM signal; digitalWrite outputs HIGH or LOW.*

6. **What is the bootloader in Arduino?**  
   *A small program that allows the microcontroller to be programmed via USB.*

7. **What is the purpose of decoupling capacitors on the board?**  
   *To reduce voltage spikes and stabilize the power supply.*

8. **How does the microcontroller execute code?**  
   *It fetches machine instructions from Flash memory and executes them sequentially.*

9. **Why are pull-up resistors needed in digital inputs?**  
   *To ensure the pin stays at HIGH when not actively driven LOW.*

10. **Can you interface 3.3V devices with Arduino Uno?**  
   *Yes, but level shifters may be needed to avoid damaging the device.*

---


# LCD 16x2 with I2C module

---

## 🔹 **Basic Questions (10 Q&A)**

1. **What is a 16x2 LCD?**  
   *It’s a liquid crystal display that can show 2 lines with 16 characters each.*

2. **What does I2C in LCD 16x2 I2C mean?**  
   *I2C stands for Inter-Integrated Circuit, a serial communication protocol used to reduce pin usage.*

3. **How many pins are required to interface a regular 16x2 LCD?**  
   *Usually 12–16 pins (depending on backlight), but only 4 data pins are needed in 4-bit mode.*

4. **How many pins are used when using LCD with I2C module?**  
   *Only 2 pins (SDA and SCL) are needed for data communication.*

5. **What are SDA and SCL?**  
   *SDA is Serial Data Line, and SCL is Serial Clock Line used in I2C communication.*

6. **Why is LCD 16x2 I2C preferred over normal LCD?**  
   *Because it reduces wiring complexity by using only 2 data lines instead of 12+.*

7. **Can LCD 16x2 display characters and numbers?**  
   *Yes, it can display letters, numbers, and custom characters.*

8. **What is the default I2C address of a 16x2 LCD module?**  
   *Typically 0x27 or 0x3F.*

9. **What is the function of the potentiometer on the I2C module?**  
   *To adjust the contrast of the display.*

10. **What type of display technology does 16x2 LCD use?**  
   *It uses HD44780 LCD controller technology.*

---

## 🔹 **Pin and Communication Related (10 Q&A)**

1. **Which Arduino pins are used for I2C communication?**  
   *A4 (SDA) and A5 (SCL) on Arduino Uno.*

2. **What is the significance of the EN and RS pins on the LCD (without I2C)?**  
   *EN is Enable; RS is Register Select (command or data).*

3. **What does the RW pin do on the LCD?**  
   *RW selects Read or Write mode; typically tied to ground for writing only.*

4. **How does I2C communicate with LCD internally?**  
   *Via a PCF8574 I/O expander chip on the I2C module that controls LCD pins.*

5. **How is the LCD powered?**  
   *Via VCC and GND (typically 5V logic).*

6. **What are the I2C addresses and how are they determined?**  
   *They’re 7-bit values (like 0x27), determined by jumper settings on the I2C module.*

7. **Can multiple I2C devices be connected to Arduino?**  
   *Yes, using unique addresses for each device on the same SDA/SCL lines.*

8. **What are the roles of the D4-D7 pins in 4-bit LCD mode?**  
   *They carry data to the LCD when using 4-bit mode (common for Arduino LCDs).*

9. **What does the 'backlight' control do on I2C modules?**  
   *Turns the display’s backlight on or off for visibility.*

10. **What happens if SDA and SCL are swapped?**  
   *The display will not function, and I2C communication will fail.*

---

## 🔹 **Practical Use / Application (10 Q&A)**

1. **Which library is commonly used to work with 16x2 I2C LCD in Arduino IDE?**  
   *`LiquidCrystal_I2C.h`*

2. **How do you initialize the LCD in Arduino code?**  
   ```cpp
   LiquidCrystal_I2C lcd(0x27, 16, 2);
   lcd.begin();
   ```

3. **How do you display text on the LCD?**  
   ```cpp
   lcd.setCursor(0, 0);  
   lcd.print("Hello World");
   ```

4. **How can you clear the screen using code?**  
   *Using `lcd.clear();`*

5. **What happens if the I2C address is wrong in the code?**  
   *The LCD will not display anything.*

6. **Can you scroll text on a 16x2 LCD?**  
   *Yes, using `lcd.scrollDisplayLeft()` and `lcd.scrollDisplayRight()`.*

7. **Can you create custom characters on a 16x2 LCD?**  
   *Yes, using `lcd.createChar()` with an 8-byte array.*

8. **How to check the I2C address of your LCD?**  
   *Using an I2C scanner code that prints addresses to Serial Monitor.*

9. **Can a 16x2 LCD I2C be used in battery-powered systems?**  
   *Yes, but it consumes power, so backlight control is useful to save power.*

10. **How to turn off the backlight of the LCD using code?**  
   *Use `lcd.noBacklight();` and `lcd.backlight();` to control it.*

---

## 🔹 **Advanced / EC-Related Questions (10 Q&A)**

1. **What is the I2C protocol speed typically used with LCDs?**  
   *Standard speed is 100 kHz, but it can go up to 400 kHz.*

2. **What is the function of the PCF8574 IC on the I2C module?**  
   *It acts as an 8-bit GPIO expander to control LCD pins via I2C.*

3. **Why is pull-up resistance used in I2C communication?**  
   *To keep the data and clock lines HIGH by default.*

4. **What is the limitation of using I2C for LCD?**  
   *It is slightly slower than direct parallel connection due to serial data.*

5. **What kind of bus is I2C — unidirectional or bidirectional?**  
   *Bidirectional – both master and slave can communicate.*

6. **What happens if you use two I2C LCDs with the same address?**  
   *They will conflict and neither may work properly.*

7. **How does the LCD create characters using pixels?**  
   *Each character is a matrix of 5x8 pixels on a 16x2 LCD.*

8. **What is the purpose of the enable (EN) signal in LCD operation?**  
   *To latch data sent to the LCD; it tells the LCD to process the input.*

9. **How does an LCD distinguish between command and data?**  
   *Using the RS (Register Select) pin — RS = 0 for command, RS = 1 for data.*

10. **Can LCD I2C modules work with 3.3V boards like ESP32?**  
   *Yes, many do, but voltage compatibility must be checked.*

---


# Ultrasonic Sensor (HC-SR04)

---

## 🔹 **Basic Questions (10 Q&A)**

1. **What is an ultrasonic sensor?**  
   *A sensor that uses ultrasonic sound waves to measure the distance to an object.*

2. **What is the working principle of the HC-SR04 ultrasonic sensor?**  
   *It emits ultrasonic waves and calculates distance based on the time taken for the echo to return.*

3. **What is the frequency of the sound wave used in HC-SR04?**  
   *Typically 40 kHz.*

4. **What are the four pins of the HC-SR04 sensor?**  
   *VCC, Trig (Trigger), Echo, and GND.*

5. **What voltage does the HC-SR04 operate on?**  
   *5V DC.*

6. **What is the range of the HC-SR04 sensor?**  
   *2 cm to 400 cm (4 meters).*

7. **What is the resolution of the sensor?**  
   *Typically 3 mm.*

8. **How do you trigger a measurement?**  
   *By sending a 10μs HIGH pulse to the Trig pin.*

9. **What does the Echo pin do?**  
   *It goes HIGH for a duration proportional to the distance to the object.*

10. **Is ultrasonic sensing affected by color or transparency?**  
   *No, it works based on sound, not light — so it is not affected by color or transparency.*

---

## 🔹 **Pin and Communication Related (10 Q&A)**

1. **Which pin on Arduino is used for Trig and Echo?**  
   *Any digital I/O pin can be used, defined in code.*

2. **Why is a 10μs pulse sent to the Trig pin?**  
   *It tells the sensor to start sending out 8 ultrasonic pulses.*

3. **What happens after the 10μs pulse is sent?**  
   *The Echo pin goes HIGH until the reflected wave is received.*

4. **How is the distance calculated in Arduino code?**  
   ```cpp
   distance = duration * 0.034 / 2;
   ```

5. **Why is the duration divided by 2 in the calculation?**  
   *Because the signal travels to the object and back — so only half the time is one-way.*

6. **How do you measure pulse duration on the Echo pin?**  
   *Using `pulseIn(EchoPin, HIGH);` in Arduino.*

7. **Can the HC-SR04 work with 3.3V boards like ESP32?**  
   *VCC needs 5V, but logic pins might require a voltage divider or level shifter for 3.3V boards.*

8. **What happens if the Echo pin is directly connected to a 3.3V microcontroller?**  
   *It may damage the pin or give inaccurate readings due to voltage mismatch.*

9. **Can the sensor work in low light or dark areas?**  
   *Yes, because it uses sound waves, not light.*

10. **What is the angle of detection of HC-SR04?**  
   *Approximately 15 degrees.*

---

## 🔹 **Practical Use / Application (10 Q&A)**

1. **What are common uses of ultrasonic sensors in robotics?**  
   *Obstacle avoidance, distance measurement, water level sensing.*

2. **Can you use multiple ultrasonic sensors at once?**  
   *Yes, but they should be triggered at different times to avoid interference.*

3. **What happens if two sensors are triggered at the same time?**  
   *Their signals may interfere, causing inaccurate measurements.*

4. **How is the ultrasonic sensor used in obstacle-avoiding robots?**  
   *By measuring distance to nearby objects and changing direction if too close.*

5. **What kind of surfaces can cause unreliable readings?**  
   *Soft or angled surfaces that absorb or deflect sound waves.*

6. **Can ultrasonic sensors be used underwater?**  
   *Not HC-SR04 — specialized waterproof ultrasonic sensors are used underwater.*

7. **Is HC-SR04 suitable for measuring human presence?**  
   *Only in terms of distance, not detection — it’s not as reliable as PIR sensors for motion detection.*

8. **How to improve accuracy in measurements?**  
   *Take multiple readings and average them.*

9. **How can you test the HC-SR04 sensor?**  
   *Upload a simple Arduino sketch and print distance values to the Serial Monitor.*

10. **Can it be used for measuring liquid levels?**  
   *Yes, but should be mounted above the liquid surface in an enclosed setup.*

---

## 🔹 **Advanced / EC-Related Questions (10 Q&A)**

1. **What is the speed of sound in air used for distance calculation?**  
   *Approximately 343 meters per second (at room temperature).*

2. **What could cause inaccurate readings in an ultrasonic sensor?**  
   *Echo interference, soft materials, angled surfaces, temperature changes.*

3. **What happens if there's no object in front of the sensor?**  
   *The Echo pin will time out and return maximum distance.*

4. **Why is temperature important in ultrasonic sensors?**  
   *Because the speed of sound varies with temperature, affecting accuracy.*

5. **What is time-of-flight in ultrasonic sensing?**  
   *The time it takes for sound to travel to an object and back.*

6. **What role does the microcontroller play in ultrasonic measurement?**  
   *It sends the trigger signal, measures pulse duration on Echo, and calculates distance.*

7. **Can an ultrasonic sensor be used in noisy environments?**  
   *May give errors due to interference, since it works on sound waves.*

8. **What are alternatives to ultrasonic sensors for distance measurement?**  
   *Infrared sensors, LiDAR, ToF (Time of Flight) sensors.*

9. **How can interference between sensors be minimized?**  
   *By triggering them sequentially with delay.*

10. **What is the duty cycle of the transmitted ultrasonic pulse?**  
   *Typically an 8-cycle burst at 40 kHz.*

---

# L298N Motor Driver Module

---

## 🔹 **Basic Questions (10 Q&A)**

1. **What is an L298N motor driver?**  
   *It's a dual H-bridge motor driver IC/module used to control the speed and direction of two DC motors or one stepper motor.*

2. **What is an H-bridge?**  
   *An H-bridge is an electronic circuit that enables a voltage to be applied across a load in either direction — used for motor direction control.*

3. **What type of motors can be controlled using L298N?**  
   *Two DC motors or one stepper motor.*

4. **How many channels does L298N have?**  
   *Two output channels — OUT1/OUT2 and OUT3/OUT4.*

5. **What is the operating voltage range of L298N?**  
   *Typically 5V to 35V for motor supply and 5V logic input.*

6. **What is the maximum current output per channel?**  
   *Around 2A continuous (with heat sink), though practically 1A is safer.*

7. **What does the ENA and ENB pin do?**  
   *ENA and ENB are enable pins for motors A and B respectively — used to control whether a motor is active.*

8. **Can you control motor speed with L298N?**  
   *Yes, by providing a PWM signal to the ENA or ENB pin.*

9. **Is L298N capable of bidirectional motor control?**  
   *Yes, it allows motors to rotate both clockwise and counterclockwise.*

10. **Why is a heat sink provided on the L298N module?**  
   *To dissipate heat generated when driving motors at high current.*

---

## 🔹 **Pin and Communication Related (10 Q&A)**

1. **What are IN1, IN2, IN3, and IN4 used for?**  
   *They are control inputs used to decide the direction of each motor.*

2. **Which pins are connected to the motors?**  
   *OUT1 & OUT2 for Motor A, OUT3 & OUT4 for Motor B.*

3. **What does the 12V terminal do?**  
   *It provides power to the motors connected to the module.*

4. **What is the purpose of the 5V pin on the module?**  
   *It provides logic voltage — when jumper is connected, it supplies 5V from onboard regulator.*

5. **What happens if the jumper is removed from the 5V EN pin?**  
   *Then you must supply 5V externally for logic control.*

6. **How do you connect the module to an Arduino?**  
   *Use IN1–IN4 as digital pins, ENA/ENB as PWM pins, and power connections.*

7. **How is speed controlled in the motor?**  
   *Using `analogWrite()` to the ENA/ENB pins (PWM signal).*

8. **Can the L298N module be used with Raspberry Pi?**  
   *Yes, with proper GPIO pin connections and voltage precautions.*

9. **What is the GND pin connected to?**  
   *Common ground with Arduino or power supply.*

10. **What is the typical wiring for one motor on L298N?**  
   *Connect motor wires to OUT1 and OUT2, IN1 and IN2 to Arduino, ENA to PWM pin.*

---

## 🔹 **Practical Use / Application (10 Q&A)**

1. **How can L298N be used in a robot?**  
   *For driving the left and right motors in forward/reverse directions.*

2. **What is the use of PWM in motor control?**  
   *To vary the average voltage and thus the speed of the motor.*

3. **Can the L298N control a stepper motor?**  
   *Yes, one stepper motor using 4 control inputs.*

4. **What happens if both IN1 and IN2 are LOW?**  
   *The motor will be in brake mode or stopped.*

5. **How do you reverse a motor direction?**  
   *By changing the logic states of IN1 and IN2 (e.g., HIGH-LOW becomes LOW-HIGH).*

6. **What precautions are needed when powering L298N from battery?**  
   *Ensure current and voltage ratings match motor and module requirements.*

7. **Can L298N drive a servo motor?**  
   *Not directly. Servo motors are better controlled using PWM signals.*

8. **Why is it important to connect motor power and logic grounds together?**  
   *To establish a common reference point for signals.*

9. **Can you run both motors at different speeds?**  
   *Yes, by sending different PWM signals to ENA and ENB.*

10. **What happens if ENA/ENB is not connected?**  
   *The motor won't run — enable pins must be HIGH or controlled by PWM.*

---

## 🔹 **Advanced / EC-Related Questions (10 Q&A)**

1. **What is the internal structure of L298N?**  
   *It contains two full H-bridge circuits made of transistors for controlling motor direction.*

2. **Why are flyback diodes important in motor driver circuits?**  
   *To protect the driver IC from voltage spikes caused by back EMF from motors.*

3. **Does the L298N have internal protection diodes?**  
   *Yes, but external ones are often added for better protection.*

4. **What is back EMF?**  
   *A voltage generated in the opposite direction when a motor slows down suddenly.*

5. **What’s the logic voltage threshold for L298N inputs?**  
   *Typically around 2.3V for HIGH, 1.5V or lower for LOW.*

6. **Why is the motor voltage supply separated from the logic supply?**  
   *To safely handle higher voltage for motors without damaging the logic control.*

7. **How does the L298N handle overcurrent?**  
   *It doesn't have built-in current limiting — external current limiting or fuses should be used.*

8. **What is the power dissipation concern in L298N?**  
   *High current causes heat due to internal resistance; hence, a heat sink is needed.*

9. **Why is L298N less efficient compared to modern drivers?**  
   *Because it uses bipolar transistors which have more voltage drop and heat loss than MOSFET-based drivers.*

10. **Name some alternatives to L298N.**  
   *L293D, DRV8833, TB6612FNG — these offer higher efficiency and smaller size.*

---

# Servo Motor 

---

## 🔹 **Basic Questions (10 Q&A)**

1. **What is a servo motor?**  
   *A servo motor is a rotary actuator that allows precise control of angular position, speed, and acceleration.*

2. **How is a servo motor different from a DC motor?**  
   *Servo motors provide precise position control with feedback, while DC motors do not inherently have feedback mechanisms.*

3. **What are the common types of servo motors?**  
   *Positional Rotation Servo, Continuous Rotation Servo, and Linear Servo.*

4. **What is the working voltage of a standard hobby servo?**  
   *Typically 4.8V to 6V.*

5. **What are the three wires in a servo motor?**  
   *Power (usually red), Ground (usually black/brown), and Control (usually yellow/orange).*

6. **What is the range of rotation of a standard servo motor?**  
   *Usually 0° to 180°, some can go from 90° to -90°.*

7. **Is a servo motor analog or digital?**  
   *It can be either. Hobby servos are usually analog, while industrial ones can be digital.*

8. **What controls the position of a servo motor?**  
   *A Pulse Width Modulated (PWM) signal.*

9. **What happens if no PWM signal is given to the servo?**  
   *It won’t move, or it might return to a default or last known position.*

10. **What is the function of the gearbox in a servo motor?**  
   *To increase torque and reduce speed, allowing finer control.*

---

## 🔹 **Pin and Communication Related (10 Q&A)**

1. **Which microcontroller pin type is used to control a servo motor?**  
   *A digital PWM pin.*

2. **What PWM signal range is typical for servo motors?**  
   *Pulse width typically ranges from 1ms (0°) to 2ms (180°), repeated every 20ms.*

3. **How many degrees does a 1.5ms PWM signal represent?**  
   *Usually around 90°, the center position.*

4. **Can you control a servo with analogWrite()?**  
   *Yes, in Arduino, it’s abstracted using libraries like `Servo.h` which handles PWM generation.*

5. **What happens if you continuously send the same angle signal?**  
   *The servo holds its position with torque.*

6. **Can a servo motor be powered directly from the Arduino?**  
   *For small servos, yes. For larger servos, use an external 5V power supply.*

7. **Why do we connect servo ground to Arduino ground?**  
   *To maintain a common reference voltage and prevent erratic behavior.*

8. **What is the role of the control wire in a servo?**  
   *It receives PWM signals that set the motor’s position.*

9. **Can we use GPIO pins from Raspberry Pi to control a servo motor?**  
   *Yes, with appropriate libraries like RPi.GPIO or pigpio.*

10. **What is the maximum number of servos an Arduino UNO can handle?**  
   *Up to 12 with the `Servo.h` library, though performance may drop.*

---

## 🔹 **Practical Use / Application (10 Q&A)**

1. **Where are servo motors used in robotics?**  
   *In arms, grippers, pan-tilt cameras, and joints that require precise motion.*

2. **Can a servo motor be used for continuous rotation like a wheel?**  
   *Yes, but only if it's a continuous rotation servo.*

3. **What causes jittering in a servo motor?**  
   *Unstable PWM signals, inadequate power, or noisy inputs.*

4. **What is torque in servo motors?**  
   *The rotational force the servo can exert — usually rated in kg-cm.*

5. **What will happen if a servo is asked to rotate beyond its mechanical limit?**  
   *It may stall, jitter, or get damaged.*

6. **How can you increase the torque of a servo system?**  
   *Use gear reduction or choose a servo with higher torque rating.*

7. **Can you daisy chain multiple servo motors?**  
   *Not directly — each needs a separate control signal, but the power and ground can be shared.*

8. **What happens if the power supply to a servo is insufficient?**  
   *The servo may behave erratically, lose torque, or reset the microcontroller.*

9. **Why is a capacitor sometimes added near the servo?**  
   *To smooth power fluctuations and prevent voltage drops.*

10. **How can you avoid damaging your servo in code?**  
   *Avoid sending commands rapidly, exceeding range, or stalling under high load.*

---

## 🔹 **Advanced / EC-Related Questions (10 Q&A)**

1. **What is the feedback mechanism in a servo motor?**  
   *A potentiometer connected to the output shaft provides positional feedback.*

2. **How is error correction handled in servo motors?**  
   *The internal control circuit compares actual vs desired position and adjusts accordingly (PID control logic).*

3. **Why are servo motors preferred in precision control applications?**  
   *Because of their ability to maintain a specific angle with feedback.*

4. **What is deadband in a servo motor?**  
   *A small range of PWM input where the servo does not respond to prevent oscillations.*

5. **Can we reverse the direction of servo like DC motor?**  
   *No — direction is implied by target angle within its range.*

6. **What type of signal is needed for servo control — analog or digital?**  
   *Digital PWM signal.*

7. **Why might you use an external power source with servo motors?**  
   *To provide enough current without overloading the microcontroller.*

8. **Explain duty cycle in PWM with respect to servo control.**  
   *Duty cycle defines the ON duration of the signal — it determines the angle position.*

9. **What is the resolution of a standard servo motor?**  
   *Usually around 1° to 2°, depending on internal gearing and electronics.*

10. **How does the potentiometer in servo act as a position sensor?**  
   *It changes resistance as the shaft rotates, feeding back voltage proportional to the angle.*

---

# IR Sensor

---

## 🔹 **1. Basic Questions (10 Q&A)**

1. **What is an IR sensor?**  
   *An Infrared (IR) sensor is an electronic device that detects IR radiation from objects and determines presence or motion.*

2. **What are the two types of IR sensors?**  
   *Active IR sensors (emitter and receiver) and Passive IR sensors (PIR).*

3. **What components are inside an active IR sensor?**  
   *An IR LED (emitter) and a photodiode or phototransistor (receiver).*

4. **What principle does an IR proximity sensor work on?**  
   *Reflection — when an object is near, the emitted IR light reflects back to the receiver.*

5. **What is the typical range of a basic IR sensor?**  
   *From a few centimeters up to about 80 cm, depending on the surface and power.*

6. **Does IR sensor work well on black surfaces?**  
   *No, black surfaces absorb IR radiation and reduce reflection, making detection harder.*

7. **Can IR sensors detect heat?**  
   *Passive IR (PIR) sensors detect body heat; active IR proximity sensors do not.*

8. **What is the output of an IR sensor?**  
   *Usually digital — HIGH when no object, LOW when object is detected (can vary based on sensor).*

9. **Is it affected by sunlight?**  
   *Yes, ambient IR from sunlight can interfere with the sensor’s readings.*

10. **What is the operating voltage of a typical IR sensor module?**  
   *3.3V to 5V.*

---

## 🔹 **2. Pin and Communication Related (10 Q&A)**

1. **How many pins does a basic IR sensor module have?**  
   *Usually 3 — VCC, GND, and OUT.*

2. **What does the OUT pin of an IR sensor do?**  
   *It gives a digital signal depending on whether an object is detected.*

3. **Can we connect the OUT pin to an analog pin?**  
   *Yes, but it will be used as a digital input.*

4. **What happens to OUT pin when an object is detected?**  
   *It typically goes LOW (0V), depending on the sensor’s configuration.*

5. **How do we calibrate an IR sensor’s sensitivity?**  
   *Using a small potentiometer on the module.*

6. **Can an IR sensor be interfaced with a Raspberry Pi?**  
   *Yes, using GPIO pins and reading digital input.*

7. **Why do we need a common GND with Arduino or Raspberry Pi?**  
   *To ensure a common reference voltage for accurate signal reading.*

8. **What if IR sensor OUT pin is floating?**  
   *The controller might receive noisy or incorrect values.*

9. **Is PWM involved in IR sensors?**  
   *No, most IR proximity sensors use simple HIGH/LOW digital output.*

10. **Can we use multiple IR sensors in one circuit?**  
   *Yes, as long as each sensor is connected to a different digital pin.*

---

## 🔹 **3. Application & Practical Use (10 Q&A)**

1. **Where are IR sensors commonly used?**  
   *Obstacle detection in robots, line following, hand sanitizer dispensers, automatic doors.*

2. **How does an IR sensor help in a line follower robot?**  
   *It detects the difference between black and white surfaces based on IR reflection.*

3. **Why is IR sensor used in robotic obstacle avoidance?**  
   *To detect nearby objects and change direction to avoid collisions.*

4. **Can IR sensors detect transparent objects like glass?**  
   *Not reliably — reflection from transparent surfaces is minimal.*

5. **How do you avoid false triggers due to sunlight?**  
   *Use IR sensors with modulation (e.g., 38kHz IR signal).*

6. **Can IR sensors detect distance?**  
   *Not accurately — they are good for presence detection, not precise distance.*

7. **Why do some IR sensors have an onboard LED?**  
   *To indicate when an object is detected — useful for debugging.*

8. **Can IR sensors be used for speed detection?**  
   *Yes, using two sensors spaced apart and timing object passage.*

9. **Do IR sensors consume a lot of power?**  
   *No, they are generally low-power devices.*

10. **What’s a common issue in IR sensor-based applications?**  
   *Interference from external IR sources and incorrect sensitivity settings.*

---

## 🔹 **4. EC Department-Oriented Advanced Questions (10 Q&A)**

1. **What is the wavelength of IR radiation used in these sensors?**  
   *Usually around 850 nm to 950 nm.*

2. **What type of diode is used in an IR receiver?**  
   *A photodiode or phototransistor.*

3. **What is the working principle of a photodiode?**  
   *It generates current when exposed to IR light — used in reverse bias mode.*

4. **How does ambient light affect IR sensor performance?**  
   *It introduces noise and false signals — modulated IR helps overcome this.*

5. **Why use a modulated IR signal in advanced systems?**  
   *To differentiate between the sensor signal and ambient IR light.*

6. **How can you increase the detection range of an IR sensor?**  
   *Use a more powerful IR LED or a focusing lens.*

7. **What is the response time of an IR sensor?**  
   *Typically a few milliseconds.*

8. **What role does the photodiode's spectral sensitivity play?**  
   *It defines the wavelength range the diode can detect efficiently.*

9. **How can filtering improve IR sensor accuracy?**  
   *By rejecting unwanted wavelengths and noise from sunlight or lamps.*

10. **How does surface texture affect IR reflection?**  
   *Smooth, light-colored surfaces reflect better, while rough or dark surfaces absorb IR.*

---
# Touch Sensor

---

## 🔹 **1. Basic Questions (10 Q&A)**

1. **What is a touch sensor?**  
   *A touch sensor detects physical contact or proximity of a finger or object and converts it into an electrical signal.*

2. **What types of touch sensors exist?**  
   *Capacitive and resistive touch sensors are the most common types.*

3. **Which type is most commonly used in hobby electronics?**  
   *Capacitive touch sensors, like the TTP223 module.*

4. **How does a capacitive touch sensor work?**  
   *It senses a change in capacitance when a conductive object (like a finger) comes near the sensor pad.*

5. **What is the output of a capacitive touch sensor?**  
   *A digital HIGH or LOW signal depending on whether it is being touched.*

6. **Is it necessary to apply pressure on a capacitive sensor?**  
   *No, just contact or proximity is enough — no pressure needed.*

7. **Does the sensor work with gloves on?**  
   *Not always — depending on glove material and thickness, capacitance change might not be detected.*

8. **What material is used to make the touchpad?**  
   *Usually copper or other conductive material with a protective layer.*

9. **Can touch sensors detect multiple touches?**  
   *Basic modules can’t, but advanced touchscreen controllers can detect multi-touch.*

10. **What is the operating voltage of the TTP223 touch sensor module?**  
   *Typically 2V to 5.5V (often powered by 5V in Arduino projects).*

---

## 🔹 **2. Pins & Communication Related (10 Q&A)**

1. **How many pins are there in a TTP223 touch sensor module?**  
   *Three — VCC, GND, and SIG (output).*

2. **What type of signal is generated by the SIG pin?**  
   *Digital — HIGH when touched, LOW when untouched (or vice versa depending on mode).*

3. **Can you change the output mode of the TTP223 sensor?**  
   *Yes, by soldering jumpers on the back of the module (toggle or momentary modes).*

4. **Does the touch sensor need an analog or digital pin in microcontrollers?**  
   *A digital input pin.*

5. **What is the typical default output state when not touched?**  
   *LOW or HIGH depending on configuration — most default to LOW when not touched.*

6. **Is any library required to use a touch sensor with Arduino?**  
   *No — just read the digital input pin.*

7. **How fast does the sensor respond to touch?**  
   *Usually within 60–200 milliseconds.*

8. **Can we connect multiple touch sensors to one microcontroller?**  
   *Yes, using separate digital input pins for each.*

9. **Is touch sensor output active-high or active-low?**  
   *Depends on configuration, but TTP223 is usually active-high.*

10. **Can we use touch sensor as an interrupt source in Arduino?**  
   *Yes, connect it to an interrupt-capable digital pin and use attachInterrupt().*

---

## 🔹 **3. Applications & Practical Use (10 Q&A)**

1. **Where are touch sensors commonly used?**  
   *Smartphones, elevators, lamps, doorbells, interactive kiosks, and robotics.*

2. **Why are touch sensors preferred over mechanical buttons?**  
   *No moving parts, more reliable, sleek design, and less wear over time.*

3. **Can touch sensors be hidden under plastic or glass?**  
   *Yes, capacitive sensors can work through non-conductive layers.*

4. **What is a practical robotics application of a touch sensor?**  
   *Triggering commands, activating robots, or detecting human interaction.*

5. **Can a touch sensor work underwater?**  
   *Usually not — water can interfere with capacitance detection.*

6. **Can the sensitivity of a touch sensor be adjusted?**  
   *Yes, in some modules like TTP223 by changing resistor or capacitor values.*

7. **Can a touch sensor be used to toggle a state (ON/OFF)?**  
   *Yes, when configured in toggle mode.*

8. **How to debounce a touch sensor in code?**  
   *Introduce small delays (e.g., 50 ms) or use software debouncing logic.*

9. **Can we use a touch sensor as a password input system?**  
   *Yes — multiple sensors can be arranged in a keypad fashion.*

10. **How is a touch sensor used in lamps?**  
   *Touching toggles the state of the lamp (ON/OFF or brightness levels).*

---

## 🔹 **4. EC-Oriented Advanced Concepts (10 Q&A)**

1. **What is capacitance?**  
   *Capacitance is the ability of a system to store electrical charge — key to capacitive touch sensors.*

2. **What changes in a touch sensor when you touch it?**  
   *The capacitance of the sensor pad increases due to the body’s conductivity.*

3. **What kind of IC is used in TTP223?**  
   *A TTP223 capacitive touch controller IC.*

4. **What’s the function of the pull-down resistor in touch sensor output?**  
   *To ensure defined LOW state when the sensor is not active.*

5. **How does a touch sensor distinguish between real and false touches?**  
   *It uses capacitance thresholds and filters to avoid false positives.*

6. **Why do we need filtering in touch sensors?**  
   *To reject noise and ensure only valid touches are registered.*

7. **What is hysteresis in a capacitive touch sensor?**  
   *The difference in capacitance thresholds between touch and release to avoid flickering.*

8. **What is a touch debounce circuit?**  
   *A circuit or logic that filters out rapid on-off transitions due to signal noise or bouncing.*

9. **What frequency range does the touch detection system typically operate at?**  
   *Typically in the low kHz range (under 100 kHz) for capacitive systems.*

10. **Why is ground shielding important in capacitive sensor layouts?**  
   *To prevent unintended capacitance and improve sensor reliability.*

---
