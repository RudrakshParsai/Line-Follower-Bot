// Encoder with PID and IR Array
// This code implements a left-preferring line-following strategy.
// The robot is biased to favor left turns when the line is lost or when the position is uncertain.
// This approach can be useful in environments with ambiguous line markings or intersections.
// The left bias is achieved by adjusting the error calculation and decision logic.

// Motor Driver Pins
const int LEFT_MOTOR_IN1 = 9;
const int LEFT_MOTOR_IN2 = 10;
const int RIGHT_MOTOR_IN1 = 11;
const int RIGHT_MOTOR_IN2 = 12;
const int LEFT_MOTOR_EN = 13;
const int RIGHT_MOTOR_EN = 14;

// Encoder Pins
const int LEFT_ENCODER_A = 15;
const int LEFT_ENCODER_B = 16;
const int RIGHT_ENCODER_A = 17;
const int RIGHT_ENCODER_B = 18;

// Line Following IR Sensors (5 sensors)
const int IR_LEFT_2 = 2;
const int IR_LEFT_1 = 3;
const int IR_CENTER = 4;
const int IR_RIGHT_1 = 5;
const int IR_RIGHT_2 = 6;

// Encoder Variables
volatile long leftEncoderCount = 0;
volatile long rightEncoderCount = 0;
long lastLeftCount = 0;
long lastRightCount = 0;

// PID Constants
float Kp = 0.5;
float Ki = 0.01;
float Kd = 0.1;
float speedKp = 0.1; // Proportional gain for speed control

// Desired Speed
const int desiredSpeed = 200;

// Sensor Weights for Error Calculation
const int WEIGHT_LEFT_2 = -2;
const int WEIGHT_LEFT_1 = -1;
const int WEIGHT_CENTER = 0;
const int WEIGHT_RIGHT_1 = 1;
const int WEIGHT_RIGHT_2 = 2;

// Global Variables
float error = 0;
float lastError = 0;
float integral = 0;
float derivative = 0;

void setup() {
  Serial.begin(9600);

  // Configure motor control pins
  pinMode(LEFT_MOTOR_IN1, OUTPUT);
  pinMode(LEFT_MOTOR_IN2, OUTPUT);
  pinMode(RIGHT_MOTOR_IN1, OUTPUT);
  pinMode(RIGHT_MOTOR_IN2, OUTPUT);
  pinMode(LEFT_MOTOR_EN, OUTPUT);
  pinMode(RIGHT_MOTOR_EN, OUTPUT);

  // Configure encoder pins
  pinMode(LEFT_ENCODER_A, INPUT_PULLUP);
  pinMode(LEFT_ENCODER_B, INPUT_PULLUP);
  pinMode(RIGHT_ENCODER_A, INPUT_PULLUP);
  pinMode(RIGHT_ENCODER_B, INPUT_PULLUP);

  // Configure IR sensor pins
  pinMode(IR_LEFT_2, INPUT_PULLUP);
  pinMode(IR_LEFT_1, INPUT_PULLUP);
  pinMode(IR_CENTER, INPUT_PULLUP);
  pinMode(IR_RIGHT_1, INPUT_PULLUP);
  pinMode(IR_RIGHT_2, INPUT_PULLUP);

  // Attach interrupts for encoder readings
  attachInterrupt(digitalPinToInterrupt(LEFT_ENCODER_A), leftEncoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENCODER_A), rightEncoderISR, CHANGE);
}

void loop() {
  // Read IR sensors
  int left2 = digitalRead(IR_LEFT_2);
  int left1 = digitalRead(IR_LEFT_1);
  int center = digitalRead(IR_CENTER);
  int right1 = digitalRead(IR_RIGHT_1);
  int right2 = digitalRead(IR_RIGHT_2);

  // Calculate line position error with a left bias
  error = (left2 * WEIGHT_LEFT_2 + 
           left1 * WEIGHT_LEFT_1 + 
           center * WEIGHT_CENTER + 
           right1 * WEIGHT_RIGHT_1 + 
           right2 * WEIGHT_RIGHT_2) - 0.5; // Add a small left bias

  // Check for line loss
  if (left2 == 0 && left1 == 0 && center == 0 && right1 == 0 && right2 == 0) {
    // Prefer left turn when line is lost
    setMotorSpeeds(-desiredSpeed, desiredSpeed);
    return;
  }

  // Calculate speed errors
  int leftSpeedError = desiredSpeed - calculateLeftMotorSpeed();
  int rightSpeedError = desiredSpeed - calculateRightMotorSpeed();

  // Calculate PID output for line following
  float linePIDOutput = (Kp * error) + (Ki * integral) + (Kd * derivative);

  // Calculate PID output for speed control
  int leftPIDOutput = calculateSpeedPIDOutput(leftSpeedError) + linePIDOutput;
  int rightPIDOutput = calculateSpeedPIDOutput(rightSpeedError) - linePIDOutput;

  // Set motor speeds
  setMotorSpeeds(leftPIDOutput, rightPIDOutput);

  // Update PID variables
  integral += error;
  derivative = error - lastError;
  lastError = error;

  delay(100); // Loop delay
}

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

int calculateLeftMotorSpeed() {
  long leftCount = leftEncoderCount - lastLeftCount;
  lastLeftCount = leftEncoderCount;
  return (leftCount / 7.0) * 60.0; // Convert to RPM using PPR of 7
}

int calculateRightMotorSpeed() {
  long rightCount = rightEncoderCount - lastRightCount;
  lastRightCount = rightEncoderCount;
  return (rightCount / 7.0) * 60.0; // Convert to RPM using PPR of 7
}

int calculateSpeedPIDOutput(int speedError) {
  // Implement PID calculation for speed control
  return speedError * speedKp; // Simple proportional control for demonstration
}

void setMotorSpeeds(int leftSpeed, int rightSpeed) {
  analogWrite(LEFT_MOTOR_EN, constrain(leftSpeed, 0, 255));
  analogWrite(RIGHT_MOTOR_EN, constrain(rightSpeed, 0, 255));
} 