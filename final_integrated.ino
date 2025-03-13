// Final Integrated Line Follower Program
// Line Following IR Sensors (5 sensors)
const int IR_LEFT_2 = 2;    // Leftmost sensor
const int IR_LEFT_1 = 3;    // Left sensor
const int IR_CENTER = 4;    // Center sensor
const int IR_RIGHT_1 = 5;   // Right sensor
const int IR_RIGHT_2 = 6;   // Rightmost sensor

// Bottom IR Sensors (4 sensors) - Separate pins for Arduino Mega
const int BOTTOM_LEFT = 19;    // Bottom left sensor
const int BOTTOM_LEFT_MID = 20; // Bottom left middle sensor
const int BOTTOM_RIGHT_MID = 21; // Bottom right middle sensor
const int BOTTOM_RIGHT = 22;    // Bottom right sensor

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

// Encoder Pins
const int LEFT_ENCODER_A = 15;   // Left encoder channel A
const int LEFT_ENCODER_B = 16;   // Left encoder channel B
const int RIGHT_ENCODER_A = 17;  // Right encoder channel A
const int RIGHT_ENCODER_B = 18;  // Right encoder channel B

// Constants
const int WALL_DISTANCE = 20;    // Distance in cm to detect wall
const int MAX_DISTANCE = 400;    // Maximum distance to measure
const int MIN_DISTANCE = 2;      // Minimum distance to measure
const int BASE_SPEED = 200;      // Base speed for motors
const int MAX_SPEED = 255;
const int MIN_SPEED = 0;
const int TURN_SPEED = 150;      // Speed for turning
const int U_TURN_SPEED = 180;    // Speed for U-turn

// PID Constants
const float Kp = 0.5;  // Proportional constant
const float Ki = 0.01; // Integral constant
const float Kd = 0.1;  // Derivative constant

// Sensor Weights for Error Calculation
const int WEIGHT_LEFT_2 = -2;
const int WEIGHT_LEFT_1 = -1;
const int WEIGHT_CENTER = 0;
const int WEIGHT_RIGHT_1 = 1;
const int WEIGHT_RIGHT_2 = 2;

// Kill Switch Pin
const int KILL_SWITCH = 16;  // Emergency stop button

// Global Variables
float error = 0;
float lastError = 0;
float integral = 0;
float derivative = 0;
long duration;
int distance;
bool wallDetected = false;
int bottomSensorStates[4] = {0, 0, 0, 0};
volatile long leftEncoderCount = 0;
volatile long rightEncoderCount = 0;
bool isUTurning = false;
unsigned long uTurnStartTime = 0;
const unsigned long U_TURN_TIMEOUT = 3000; // 3 seconds timeout for U-turn
bool isKilled = false;  // Kill switch state

void setup() {
  Serial.begin(9600);
  
  // Configure kill switch pin
  pinMode(KILL_SWITCH, INPUT_PULLUP);
  
  // Configure all sensor pins
  configureSensorPins();
  
  // Configure motor control pins
  configureMotorPins();
  
  // Configure encoder pins
  configureEncoderPins();
  
  // Attach encoder interrupts
  attachInterrupt(digitalPinToInterrupt(LEFT_ENCODER_A), leftEncoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENCODER_A), rightEncoderISR, CHANGE);
  
  Serial.println("Final Integrated Line Follower Program");
  Serial.println("------------------------------------");
}

void loop() {
  // Check kill switch first
  if (digitalRead(KILL_SWITCH) == LOW) {
    stopMotors();
    isKilled = true;
    Serial.println("EMERGENCY STOP ACTIVATED!");
    // If LCD is available, display emergency message
    lcd.clear();
    lcd.print("EMERGENCY STOP!");
    lcd.setCursor(0, 1);
    lcd.print("Reset to Start");
    
    while(digitalRead(KILL_SWITCH) == LOW) {
      delay(100); // Wait for switch release
    }
    Serial.println("Reset the Arduino to continue");
    while(true) {
      delay(1000); // Stay in emergency stop
    }
  }

  if (!isKilled) {
    // Read all sensors
    readSensors();
    
    // Check for wall
    if (wallDetected && !isUTurning) {
      handleWallDetection();
    }
    
    // Handle U-turn if in progress
    if (isUTurning) {
      handleUTurn();
    } else {
      // Normal line following
      followLine();
    }
    
    // Print debug information
    printDebugInfo();
  }
  
  // Small delay for stability
  delay(50);
}

void configureSensorPins() {
  // Line following sensors
  pinMode(IR_LEFT_2, INPUT_PULLUP);
  pinMode(IR_LEFT_1, INPUT_PULLUP);
  pinMode(IR_CENTER, INPUT_PULLUP);
  pinMode(IR_RIGHT_1, INPUT_PULLUP);
  pinMode(IR_RIGHT_2, INPUT_PULLUP);
  
  // Bottom sensors
  pinMode(BOTTOM_LEFT, INPUT_PULLUP);
  pinMode(BOTTOM_LEFT_MID, INPUT_PULLUP);
  pinMode(BOTTOM_RIGHT_MID, INPUT_PULLUP);
  pinMode(BOTTOM_RIGHT, INPUT_PULLUP);
  
  // Ultrasonic sensor
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
}

void configureMotorPins() {
  pinMode(LEFT_MOTOR_IN1, OUTPUT);
  pinMode(LEFT_MOTOR_IN2, OUTPUT);
  pinMode(RIGHT_MOTOR_IN1, OUTPUT);
  pinMode(RIGHT_MOTOR_IN2, OUTPUT);
  pinMode(LEFT_MOTOR_EN, OUTPUT);
  pinMode(RIGHT_MOTOR_EN, OUTPUT);
}

void configureEncoderPins() {
  pinMode(LEFT_ENCODER_A, INPUT_PULLUP);
  pinMode(LEFT_ENCODER_B, INPUT_PULLUP);
  pinMode(RIGHT_ENCODER_A, INPUT_PULLUP);
  pinMode(RIGHT_ENCODER_B, INPUT_PULLUP);
}

void readSensors() {
  // Read line following sensors
  int left2 = digitalRead(IR_LEFT_2);
  int left1 = digitalRead(IR_LEFT_1);
  int center = digitalRead(IR_CENTER);
  int right1 = digitalRead(IR_RIGHT_1);
  int right2 = digitalRead(IR_RIGHT_2);
  
  // Read bottom sensors
  bottomSensorStates[0] = digitalRead(BOTTOM_LEFT);
  bottomSensorStates[1] = digitalRead(BOTTOM_LEFT_MID);
  bottomSensorStates[2] = digitalRead(BOTTOM_RIGHT_MID);
  bottomSensorStates[3] = digitalRead(BOTTOM_RIGHT);
  
  // Read ultrasonic sensor
  readUltrasonicSensor();
  
  // Calculate error for line following
  error = (left2 * WEIGHT_LEFT_2 + 
           left1 * WEIGHT_LEFT_1 + 
           center * WEIGHT_CENTER + 
           right1 * WEIGHT_RIGHT_1 + 
           right2 * WEIGHT_RIGHT_2);
}

void readUltrasonicSensor() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  
  duration = pulseIn(ECHO_PIN, HIGH);
  distance = duration * 0.034 / 2;
  
  if (distance > MAX_DISTANCE || distance < MIN_DISTANCE) {
    distance = -1;
  }
  
  wallDetected = (distance <= WALL_DISTANCE && distance > 0);
}

void handleWallDetection() {
  Serial.println("Wall detected! Initiating U-turn...");
  isUTurning = true;
  uTurnStartTime = millis();
  stopMotors();
  delay(500); // Brief pause before starting U-turn
}

void handleUTurn() {
  // Check for timeout
  if (millis() - uTurnStartTime > U_TURN_TIMEOUT) {
    Serial.println("U-turn timeout! Stopping...");
    isUTurning = false;
    stopMotors();
    return;
  }
  
  // Start U-turn
  turnRight(U_TURN_SPEED);
  
  // Check if center sensor detects line
  if (digitalRead(IR_CENTER)) {
    Serial.println("Line detected during U-turn!");
    isUTurning = false;
    stopMotors();
    delay(500);
  }
}

void followLine() {
  // Calculate PID terms
  integral += error;
  derivative = error - lastError;
  
  // Calculate PID output
  float pidOutput = (Kp * error) + (Ki * integral) + (Kd * derivative);
  
  // Calculate motor speeds
  int leftSpeed = BASE_SPEED + pidOutput;
  int rightSpeed = BASE_SPEED - pidOutput;
  
  // Constrain motor speeds
  leftSpeed = constrain(leftSpeed, MIN_SPEED, MAX_SPEED);
  rightSpeed = constrain(rightSpeed, MIN_SPEED, MAX_SPEED);
  
  // Set motor speeds
  setMotorSpeeds(leftSpeed, rightSpeed);
  
  // Update last error
  lastError = error;
}

void setMotorSpeeds(int leftSpeed, int rightSpeed) {
  if (!isKilled) {
    // Left motor
    if (leftSpeed >= 0) {
      analogWrite(LEFT_MOTOR_EN, leftSpeed);
      digitalWrite(LEFT_MOTOR_IN1, HIGH);
      digitalWrite(LEFT_MOTOR_IN2, LOW);
    } else {
      analogWrite(LEFT_MOTOR_EN, -leftSpeed);
      digitalWrite(LEFT_MOTOR_IN1, LOW);
      digitalWrite(LEFT_MOTOR_IN2, HIGH);
    }
    
    // Right motor
    if (rightSpeed >= 0) {
      analogWrite(RIGHT_MOTOR_EN, rightSpeed);
      digitalWrite(RIGHT_MOTOR_IN1, HIGH);
      digitalWrite(RIGHT_MOTOR_IN2, LOW);
    } else {
      analogWrite(RIGHT_MOTOR_EN, -rightSpeed);
      digitalWrite(RIGHT_MOTOR_IN1, LOW);
      digitalWrite(RIGHT_MOTOR_IN2, HIGH);
    }
  }
}

void stopMotors() {
  analogWrite(LEFT_MOTOR_EN, 0);
  analogWrite(RIGHT_MOTOR_EN, 0);
  digitalWrite(LEFT_MOTOR_IN1, LOW);
  digitalWrite(LEFT_MOTOR_IN2, LOW);
  digitalWrite(RIGHT_MOTOR_IN1, LOW);
  digitalWrite(RIGHT_MOTOR_IN2, LOW);
}

void turnRight(int speed) {
  analogWrite(LEFT_MOTOR_EN, speed);
  analogWrite(RIGHT_MOTOR_EN, speed);
  digitalWrite(LEFT_MOTOR_IN1, HIGH);
  digitalWrite(LEFT_MOTOR_IN2, LOW);
  digitalWrite(RIGHT_MOTOR_IN1, LOW);
  digitalWrite(RIGHT_MOTOR_IN2, HIGH);
}

void printDebugInfo() {
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint > 500) {
    Serial.print("Error: ");
    Serial.print(error);
    Serial.print(" | Distance: ");
    Serial.print(distance);
    Serial.print(" cm | Wall: ");
    Serial.print(wallDetected ? "YES" : "NO");
    Serial.print(" | U-turn: ");
    Serial.print(isUTurning ? "YES" : "NO");
    Serial.print(" | Left Encoder: ");
    Serial.print(leftEncoderCount);
    Serial.print(" | Right Encoder: ");
    Serial.println(rightEncoderCount);
    lastPrint = millis();
  }
}

// Encoder ISR functions
void leftEncoderISR() {
  if (digitalRead(LEFT_ENCODER_A) == digitalRead(LEFT_ENCODER_B)) {
    leftEncoderCount++;
  } else {
    leftEncoderCount--;
  }
}

void rightEncoderISR() {
  if (digitalRead(RIGHT_ENCODER_A) == digitalRead(RIGHT_ENCODER_B)) {
    rightEncoderCount++;
  } else {
    rightEncoderCount--;
  }
} 