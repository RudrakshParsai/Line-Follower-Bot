// Simple I2C LCD Test Program
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// Initialize LCD (0x27 is the I2C address, 16x2 display)
LiquidCrystal_I2C lcd(0x27, 16, 2);

// Line Following IR Sensors (5 sensors)
const int IR_LEFT_2 = 2;    // Leftmost sensor
const int IR_LEFT_1 = 3;    // Left sensor
const int IR_CENTER = 4;    // Center sensor
const int IR_RIGHT_1 = 5;   // Right sensor
const int IR_RIGHT_2 = 6;   // Rightmost sensor

// Bottom IR Sensors (4 sensors)
const int BOTTOM_LEFT = 22;      // Bottom left (MSB)
const int BOTTOM_LEFT_MID = 23;  // Bottom left middle
const int BOTTOM_RIGHT_MID = 24; // Bottom right middle
const int BOTTOM_RIGHT = 25;     // Bottom right (LSB)

// HC-SR04 Ultrasonic Sensor Pins
const int TRIG_PIN = 7;
const int ECHO_PIN = 8;

// Variables for sensor readings
int left2, left1, center, right1, right2;
int bottomSensors[4];  // Array to store bottom sensor readings
int bottomDecimal;     // Decimal value of bottom sensors
long duration;
int distance;

// Display mode (0: Main sensors, 1: Bottom sensors)
int displayMode = 0;
unsigned long lastModeSwitch = 0;
const unsigned long MODE_SWITCH_INTERVAL = 2000; // Switch display every 2 seconds

void setup() {
  Serial.begin(9600);
  Wire.begin(); // Initialize I2C
  
  // Initialize LCD
  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.print("Sensor Test");
  delay(2000);
  
  // Configure pins
  pinMode(IR_LEFT_2, INPUT_PULLUP);
  pinMode(IR_LEFT_1, INPUT_PULLUP);
  pinMode(IR_CENTER, INPUT_PULLUP);
  pinMode(IR_RIGHT_1, INPUT_PULLUP);
  pinMode(IR_RIGHT_2, INPUT_PULLUP);
  
  // Configure bottom sensors
  pinMode(BOTTOM_LEFT, INPUT_PULLUP);
  pinMode(BOTTOM_LEFT_MID, INPUT_PULLUP);
  pinMode(BOTTOM_RIGHT_MID, INPUT_PULLUP);
  pinMode(BOTTOM_RIGHT, INPUT_PULLUP);
  
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
}

void loop() {
  // Read all sensors
  readMainSensors();
  readBottomSensors();
  readUltrasonicSensor();
  
  // Switch display mode every 2 seconds
  if (millis() - lastModeSwitch >= MODE_SWITCH_INTERVAL) {
    displayMode = !displayMode;
    lastModeSwitch = millis();
    lcd.clear();
  }
  
  // Update display based on mode
  if (displayMode == 0) {
    displayMainSensors();
  } else {
    displayBottomSensors();
  }
  
  // Print to Serial for debugging
  printDebugInfo();
  
  delay(100); // Small delay for stability
}

void readMainSensors() {
  left2 = digitalRead(IR_LEFT_2);
  left1 = digitalRead(IR_LEFT_1);
  center = digitalRead(IR_CENTER);
  right1 = digitalRead(IR_RIGHT_1);
  right2 = digitalRead(IR_RIGHT_2);
}

void readBottomSensors() {
  // Read bottom sensors (MSB to LSB order)
  bottomSensors[0] = digitalRead(BOTTOM_LEFT);      // MSB
  bottomSensors[1] = digitalRead(BOTTOM_LEFT_MID);
  bottomSensors[2] = digitalRead(BOTTOM_RIGHT_MID);
  bottomSensors[3] = digitalRead(BOTTOM_RIGHT);     // LSB
  
  // Convert binary to decimal
  bottomDecimal = 0;
  for (int i = 0; i < 4; i++) {
    bottomDecimal = (bottomDecimal << 1) | bottomSensors[i];
  }
}

void readUltrasonicSensor() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  
  duration = pulseIn(ECHO_PIN, HIGH);
  distance = duration * 0.034 / 2;
  
  if (distance > 400 || distance < 2) {
    distance = -1;
  }
}

void displayMainSensors() {
  lcd.setCursor(0, 0);
  lcd.print("IR:");
  lcd.print(left2);
  lcd.print(left1);
  lcd.print(center);
  lcd.print(right1);
  lcd.print(right2);
  
  lcd.setCursor(0, 1);
  lcd.print("Dist:");
  lcd.print(distance);
  lcd.print("cm");
}

void displayBottomSensors() {
  lcd.setCursor(0, 0);
  lcd.print("Bot:");
  // Display binary
  for (int i = 0; i < 4; i++) {
    lcd.print(bottomSensors[i]);
  }
  
  lcd.setCursor(0, 1);
  lcd.print("Dec:");
  lcd.print(bottomDecimal);
  // Add arrow to show active bits
  lcd.print(" (");
  for (int i = 0; i < bottomDecimal; i++) {
    lcd.print(">");
  }
  lcd.print(")");
}

void printDebugInfo() {
  Serial.print("Main IR: ");
  Serial.print(left2);
  Serial.print(left1);
  Serial.print(center);
  Serial.print(right1);
  Serial.print(right2);
  
  Serial.print(" | Bottom IR: ");
  for (int i = 0; i < 4; i++) {
    Serial.print(bottomSensors[i]);
  }
  Serial.print(" (");
  Serial.print(bottomDecimal);
  Serial.print(")");
  
  Serial.print(" | Distance: ");
  Serial.print(distance);
  Serial.println("cm");
} 