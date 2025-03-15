// PID Line Follower with Encoder Feedback

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

// Encoder Variables
volatile long leftEncoderCount = 0;
volatile long rightEncoderCount = 0;
long lastLeftCount = 0;
long lastRightCount = 0;

// PID Constants
float Kp = 1.0;
float Ki = 0.0;
float Kd = 0.0;
float speedKp = 0.1; // Proportional gain for speed control

// Desired Speed
const int desiredSpeed = 200;

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

  // Attach interrupts for encoder readings
  attachInterrupt(digitalPinToInterrupt(LEFT_ENCODER_A), leftEncoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENCODER_A), rightEncoderISR, CHANGE);
}

void loop() {
  // Calculate speed errors
  int leftSpeedError = desiredSpeed - calculateLeftMotorSpeed();
  int rightSpeedError = desiredSpeed - calculateRightMotorSpeed();

  // Calculate PID output
  int leftPIDOutput = calculatePIDOutput(leftSpeedError);
  int rightPIDOutput = calculatePIDOutput(rightSpeedError);

  // Set motor speeds
  setMotorSpeeds(leftPIDOutput, rightPIDOutput);

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
  return (leftCount / 20.0) * 60.0; // Convert to RPM
}

int calculateRightMotorSpeed() {
  long rightCount = rightEncoderCount - lastRightCount;
  lastRightCount = rightEncoderCount;
  return (rightCount / 20.0) * 60.0; // Convert to RPM
}

int calculatePIDOutput(int speedError) {
  // Implement PID calculation here
  return speedError * speedKp; // Simple proportional control for demonstration
}

void setMotorSpeeds(int leftSpeed, int rightSpeed) {
  analogWrite(LEFT_MOTOR_EN, constrain(leftSpeed, 0, 255));
  analogWrite(RIGHT_MOTOR_EN, constrain(rightSpeed, 0, 255));
} 