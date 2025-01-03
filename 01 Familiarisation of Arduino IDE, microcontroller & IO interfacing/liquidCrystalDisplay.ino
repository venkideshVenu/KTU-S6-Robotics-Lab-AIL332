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