#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd(0x3F, 16, 2); // Initialize the LCD with I2C address 0x3F, 16 columns, 2 rows

const int BUTTON_PIN = 7; // Pin connected to the push button
int lastState = LOW; // Stores the last state of the button
int currentState;

void setup() {
  Serial.begin(9600); // Initialize serial communication for debugging
  pinMode(BUTTON_PIN, INPUT_PULLUP); // Set the button pin as input with an internal pull-up resistor
  lcd.init(); // Initialize the LCD
  lcd.backlight(); // Turn on the LCD backlight
}

void loop() {
  currentState = digitalRead(BUTTON_PIN); // Read the current state of the button

  // Check if the button is pressed (transition from not pressed to pressed)
  if (lastState == HIGH && currentState == LOW) { 
    Serial.println("Button pressed");
    lcd.clear(); // Clear the LCD display
    lcd.setCursor(0, 0); // Set the cursor to the first row, first column
    lcd.print("TOUCHED"); // Display "TOUCHED" on the LCD
    delay(500); // Keep the message for 500ms
    lcd.clear(); // Clear the LCD display
  }

  lastState = currentState; // Update the last state
}
