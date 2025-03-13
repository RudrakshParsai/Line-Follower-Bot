// PID Line Follower Program
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

// Motor Driver Pins
const int LEFT_MOTOR_IN1 = 9;    // Left motor input 1
const int LEFT_MOTOR_IN2 = 10;   // Left motor input 2
const int RIGHT_MOTOR_IN1 = 11;  // Right motor input 1
const int RIGHT_MOTOR_IN2 = 12;  // Right motor input 2
const int LEFT_MOTOR_EN = 13;    // Left motor enable (PWM)
const int RIGHT_MOTOR_EN = 14;   // Right motor enable (PWM)

// PID Constants (to be tuned)
const float Kp = 0.5;  // Proportional constant
const float Ki = 0.01; // Integral constant
const float Kd = 0.1;  // Derivative constant

// PID Variables
float error = 0;
float lastError = 0;
float integral = 0;
float derivative = 0;

// Motor Speed Variables
const int BASE_SPEED = 200;  // Base speed for motors
const int MAX_SPEED = 255;
const int MIN_SPEED = 0;

// Sensor Weights for Error Calculation
const int WEIGHT_LEFT_2 = -2;
const int WEIGHT_LEFT_1 = -1;
const int WEIGHT_CENTER = 0;
const int WEIGHT_RIGHT_1 = 1;
const int WEIGHT_RIGHT_2 = 2;

// Bottom Sensor States
int bottomSensorStates[4] = {0, 0, 0, 0};

// Kill Switch Pin
const int KILL_SWITCH = 16;  // Emergency stop button

// Global Variables
bool isKilled = false;  // Kill switch state

void setup() {
  Serial.begin(9600);
  
  // Configure kill switch pin
  pinMode(KILL_SWITCH, INPUT_PULLUP);
  
  // Configure line following sensor pins
  pinMode(IR_LEFT_2, INPUT_PULLUP);
  pinMode(IR_LEFT_1, INPUT_PULLUP);
  pinMode(IR_CENTER, INPUT_PULLUP);
  pinMode(IR_RIGHT_1, INPUT_PULLUP);
  pinMode(IR_RIGHT_2, INPUT_PULLUP);
  
  // Configure bottom sensor pins
  pinMode(BOTTOM_LEFT, INPUT_PULLUP);
  pinMode(BOTTOM_LEFT_MID, INPUT_PULLUP);
  pinMode(BOTTOM_RIGHT_MID, INPUT_PULLUP);
  pinMode(BOTTOM_RIGHT, INPUT_PULLUP);
  
  // Configure motor control pins
  pinMode(LEFT_MOTOR_IN1, OUTPUT);
  pinMode(LEFT_MOTOR_IN2, OUTPUT);
  pinMode(RIGHT_MOTOR_IN1, OUTPUT);
  pinMode(RIGHT_MOTOR_IN2, OUTPUT);
  pinMode(LEFT_MOTOR_EN, OUTPUT);
  pinMode(RIGHT_MOTOR_EN, OUTPUT);
  
  Serial.println("PID Line Follower Program");
  Serial.println("------------------------");
}

void loop() {
  // Check kill switch first
  if (digitalRead(KILL_SWITCH) == LOW) {
    stopMotors();
    isKilled = true;
    Serial.println("EMERGENCY STOP ACTIVATED!");
    while(digitalRead(KILL_SWITCH) == LOW) {
      delay(100); // Wait for switch release
    }
    Serial.println("Reset the Arduino to continue");
    while(true) {
      delay(1000); // Stay in emergency stop
    }
  }

  if (!isKilled) {
    // Read line following sensors
    int left2 = digitalRead(IR_LEFT_2);
    int left1 = digitalRead(IR_LEFT_1);
    int center = digitalRead(IR_CENTER);
    int right1 = digitalRead(IR_RIGHT_1);
    int right2 = digitalRead(IR_RIGHT_2);
    
    // Read bottom sensors (for future use)
    bottomSensorStates[0] = digitalRead(BOTTOM_LEFT);
    bottomSensorStates[1] = digitalRead(BOTTOM_LEFT_MID);
    bottomSensorStates[2] = digitalRead(BOTTOM_RIGHT_MID);
    bottomSensorStates[3] = digitalRead(BOTTOM_RIGHT);
    
    // Calculate error using weighted sum
    error = (left2 * WEIGHT_LEFT_2 + 
             left1 * WEIGHT_LEFT_1 + 
             center * WEIGHT_CENTER + 
             right1 * WEIGHT_RIGHT_1 + 
             right2 * WEIGHT_RIGHT_2);
    
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
    
    // Set motor directions and speeds
    setMotorSpeeds(leftSpeed, rightSpeed);
    
    // Update last error
    lastError = error;
    
    // Print debug information
    printDebugInfo(leftSpeed, rightSpeed, error, pidOutput);
    
    // Small delay for stability
    delay(50);
  }
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

void printDebugInfo(int leftSpeed, int rightSpeed, float error, float pidOutput) {
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint > 500) { // Print every 500ms
    Serial.print("Error: ");
    Serial.print(error);
    Serial.print(" | PID Output: ");
    Serial.print(pidOutput);
    Serial.print(" | Left Speed: ");
    Serial.print(leftSpeed);
    Serial.print(" | Right Speed: ");
    Serial.println(rightSpeed);
    lastPrint = millis();
  }
} 