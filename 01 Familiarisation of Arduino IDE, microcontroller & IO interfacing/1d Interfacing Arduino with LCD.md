# Arduino LCD I2C Display Experiment

## Aim
To interface a 16x2 LCD with Arduino using the I2C protocol and display dynamic messages.

---

## Components Required
- Arduino board (e.g., Arduino Uno)
- 16x2 I2C LCD module (address: `0x27`)
- Breadboard
- Jumper wires

---

## Circuit Diagram
1. Connect the **SDA** pin of the I2C LCD module to the **SDA** pin of the Arduino (on Arduino Uno).
2. Connect the **SCL** pin of the I2C LCD module to the **SCL** pin of the Arduino (on Arduino Uno).
3. Connect the **VCC** pin of the I2C LCD module to the **5V** pin of the Arduino.
4. Connect the **GND** pin of the I2C LCD module to the **GND** pin of the Arduino.

---

## Video Simulation

<video width="600" controls>
  <source src="./lcd.mp4" type="video/mp4">
  Your browser does not support the video tag.
</video>

---

## Program
### Code
```cpp
#include <LiquidCrystal_I2C.h>

// Initialize the I2C LCD with address 0x27 and dimensions 16x2
LiquidCrystal_I2C lcd(0x27, 16, 2);

void setup() {
  lcd.init();          // Initialize the LCD
  lcd.backlight();     // Turn on the backlight
  lcd.setCursor(5, 0); // Set the cursor to column 5, row 0
  lcd.print("HAI!!!"); // Print "HAI!!!" on the LCD
  delay(1500);         // Wait for 1.5 seconds
}

void loop() {
  lcd.clear();          // Clear the LCD
  lcd.setCursor(5, 0);  // Set the cursor to column 5, row 0
  lcd.print("LCD-12C"); // Print "LCD-12C" on the first row
  lcd.setCursor(2, 1);  // Set the cursor to column 2, row 1
  lcd.print("HELLO WORLD !"); // Print "HELLO WORLD !" on the second row
  delay(2000);          // Wait for 2 seconds
}
```
---







## Steps to Execute
1. **Setup the Circuit**:
   - Connect the I2C LCD module to the Arduino as described in the Circuit Diagram section.

2. **Upload the Code**:
   - Open the Arduino IDE.
   - Install the `LiquidCrystal_I2C` library via the Library Manager if not already installed.
   - Copy the code into the Arduino IDE.
   - Upload the code to the Arduino board.

3. **Observe the LCD**:
   - The LCD displays "HAI!!!" for 1.5 seconds.
   - It then alternates between:
     - "LCD-12C" on the first row.
     - "HELLO WORLD !" on the second row.

---

## Notes
- Ensure the correct I2C address (`0x27`) for your LCD module. Use an I2C scanner if necessary to find the address.
- Install the `LiquidCrystal_I2C` library via the Library Manager in the Arduino IDE if it’s not already available.

---

## Troubleshooting
1. **LCD Not Displaying**:
   - Check connections for SDA, SCL, VCC, and GND.
   - Verify the I2C address using an I2C scanner.
   - Ensure the `LiquidCrystal_I2C` library is correctly installed.

2. **Backlight Off**:
   - Ensure `lcd.backlight()` is called in the setup.

---
