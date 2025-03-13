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

// HC-SR04 Ultrasonic Sensor Pins
const int TRIG_PIN = 7;
const int ECHO_PIN = 8;

// Variables for sensor readings
int left2, left1, center, right1, right2;
long duration;
int distance;

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
  
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
}

void loop() {
  // Read IR sensors
  left2 = digitalRead(IR_LEFT_2);
  left1 = digitalRead(IR_LEFT_1);
  center = digitalRead(IR_CENTER);
  right1 = digitalRead(IR_RIGHT_1);
  right2 = digitalRead(IR_RIGHT_2);
  
  // Read ultrasonic sensor
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
  
  // Update display
  lcd.clear();
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
  
  delay(500); // Update every 500ms
} 