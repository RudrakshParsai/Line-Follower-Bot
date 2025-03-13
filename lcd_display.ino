// Line Follower Data Display on I2C LCD
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

// Motor Driver Pins
const int LEFT_MOTOR_IN1 = 9;    // Left motor input 1
const int LEFT_MOTOR_IN2 = 10;   // Left motor input 2
const int RIGHT_MOTOR_IN1 = 11;  // Right motor input 1
const int RIGHT_MOTOR_IN2 = 12;  // Right motor input 2
const int LEFT_MOTOR_EN = 13;    // Left motor enable (PWM)
const int RIGHT_MOTOR_EN = 14;   // Right motor enable (PWM)

// PID Constants
const float Kp = 0.5;  // Proportional constant
const float Ki = 0.01; // Integral constant
const float Kd = 0.1;  // Derivative constant

// Global Variables
float error = 0;
float lastError = 0;
float integral = 0;
float derivative = 0;
long duration;
int distance;
bool wallDetected = false;
bool isUTurning = false;
int leftSpeed = 0;
int rightSpeed = 0;

// Display Variables
unsigned long lastDisplayUpdate = 0;
const unsigned long DISPLAY_UPDATE_INTERVAL = 500; // Update every 500ms
int displayPage = 0; // 0: Line Following, 1: Wall Detection, 2: Motor Speeds

void setup() {
  Serial.begin(9600);
  Wire.begin(); // Initialize I2C
  
  // Initialize LCD
  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.print("Line Follower");
  lcd.setCursor(0, 1);
  lcd.print("Data Display");
  delay(2000);
  
  // Configure pins
  configurePins();
}

void loop() {
  // Read sensors
  readSensors();
  
  // Update display periodically
  if (millis() - lastDisplayUpdate >= DISPLAY_UPDATE_INTERVAL) {
    updateDisplay();
    lastDisplayUpdate = millis();
  }
  
  // Check for serial commands to change display page
  if (Serial.available()) {
    char cmd = Serial.read();
    if (cmd >= '0' && cmd <= '2') {
      displayPage = cmd - '0';
      lcd.clear();
    }
  }
  
  delay(50);
}

void configurePins() {
  // Configure line following sensor pins
  pinMode(IR_LEFT_2, INPUT_PULLUP);
  pinMode(IR_LEFT_1, INPUT_PULLUP);
  pinMode(IR_CENTER, INPUT_PULLUP);
  pinMode(IR_RIGHT_1, INPUT_PULLUP);
  pinMode(IR_RIGHT_2, INPUT_PULLUP);
  
  // Configure ultrasonic sensor pins
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  
  // Configure motor control pins
  pinMode(LEFT_MOTOR_IN1, OUTPUT);
  pinMode(LEFT_MOTOR_IN2, OUTPUT);
  pinMode(RIGHT_MOTOR_IN1, OUTPUT);
  pinMode(RIGHT_MOTOR_IN2, OUTPUT);
  pinMode(LEFT_MOTOR_EN, OUTPUT);
  pinMode(RIGHT_MOTOR_EN, OUTPUT);
}

void readSensors() {
  // Read line following sensors
  int left2 = digitalRead(IR_LEFT_2);
  int left1 = digitalRead(IR_LEFT_1);
  int center = digitalRead(IR_CENTER);
  int right1 = digitalRead(IR_RIGHT_1);
  int right2 = digitalRead(IR_RIGHT_2);
  
  // Calculate error
  error = (left2 * -2 + left1 * -1 + center * 0 + right1 * 1 + right2 * 2);
  
  // Read ultrasonic sensor
  readUltrasonicSensor();
  
  // Calculate motor speeds (simplified for display)
  leftSpeed = 200 + error;
  rightSpeed = 200 - error;
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
  
  wallDetected = (distance <= 20 && distance > 0);
}

void updateDisplay() {
  lcd.clear();
  
  switch(displayPage) {
    case 0: // Line Following Data
      lcd.print("Error: ");
      lcd.print(error);
      lcd.setCursor(0, 1);
      lcd.print("Center: ");
      lcd.print(digitalRead(IR_CENTER));
      break;
      
    case 1: // Wall Detection Data
      lcd.print("Dist: ");
      lcd.print(distance);
      lcd.print("cm");
      lcd.setCursor(0, 1);
      lcd.print("Wall: ");
      lcd.print(wallDetected ? "YES" : "NO");
      break;
      
    case 2: // Motor Speeds
      lcd.print("L:");
      lcd.print(leftSpeed);
      lcd.print(" R:");
      lcd.print(rightSpeed);
      lcd.setCursor(0, 1);
      lcd.print("UTurn: ");
      lcd.print(isUTurning ? "YES" : "NO");
      break;
  }
} 