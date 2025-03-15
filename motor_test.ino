// Motor Control Test Program
// Motor Driver Pins (TBF8266FNG/L298N)
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

// Encoder Variables
volatile long leftEncoderCount = 0;
volatile long rightEncoderCount = 0;
long lastLeftCount = 0;
long lastRightCount = 0;

// Motor Speed Variables
const int MAX_SPEED = 255;
const int MIN_SPEED = 0;
int currentSpeed = 0;

// Movement Constants
const int SQUARE_SIDE = 1000;    // Encoder counts for one side
const int TURN_COUNTS = 500;     // Encoder counts for 90-degree turn

// Kill Switch Pin
const int KILL_SWITCH = 16;  // Emergency stop button

// Global Variables
bool isKilled = false;  // Kill switch state

// RPM Calculation Variables
const int ENCODER_PULSES_PER_REV = 7; // Number of encoder pulses per wheel revolution from N20 data sheet 
//https://robokits.co.in/motors/n20-metal-gear-micro-motors/n20-metal-gear-encoder-motor/ga12-n20-12v-300-rpm-all-metal-gear-micro-dc-encoder-motor-with-precious-metal-brush?srsltid=AfmBOopPFMIkYOPyrTsLFBo_L3Y9lKtT-5GI7wXfAu2s8dgCdCqxohBQ
const int WHEEL_DIAMETER_CM = 6; // Diameter of the wheel in cm
const float CM_PER_REV = WHEEL_DIAMETER_CM * 3.14159; // Circumference of the wheel

void setup() {
  Serial.begin(9600);
  
  // Configure kill switch pin
  pinMode(KILL_SWITCH, INPUT_PULLUP);
  
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
  
  Serial.println("Motor Control Test Program");
  Serial.println("-------------------------");
  
  // Wait for user input
  Serial.println("Press '1' for basic movement test");
  Serial.println("Press '2' for square path test (clockwise)");
  Serial.println("Press '3' for square path test (counter-clockwise)");
  Serial.println("Press '4' for speed control test");
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
    if (Serial.available()) {
      char command = Serial.read();
      
      switch(command) {
        case '1':
          basicMovementTest();
          break;
        case '2':
          squarePathTest(true);
          break;
        case '3':
          squarePathTest(false);
          break;
        case '4':
          speedControlTest();
          break;
        default:
          Serial.println("Invalid command");
      }
    }
    
    // Print encoder counts every second
    static unsigned long lastPrint = 0;
    if (millis() - lastPrint > 1000) {
      printEncoderCounts();
      lastPrint = millis();
    }

    // Calculate and print RPM every second
    static unsigned long lastRPMCalc = 0;
    if (millis() - lastRPMCalc > 1000) {
      calculateAndPrintRPM();
      lastRPMCalc = millis();
    }
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

// Basic movement functions
void moveForward(int speed) {
  if (!isKilled) {
    analogWrite(LEFT_MOTOR_EN, speed);
    analogWrite(RIGHT_MOTOR_EN, speed);
    digitalWrite(LEFT_MOTOR_IN1, HIGH);
    digitalWrite(LEFT_MOTOR_IN2, LOW);
    digitalWrite(RIGHT_MOTOR_IN1, HIGH);
    digitalWrite(RIGHT_MOTOR_IN2, LOW);
  }
}

void moveBackward(int speed) {
  if (!isKilled) {
    analogWrite(LEFT_MOTOR_EN, speed);
    analogWrite(RIGHT_MOTOR_EN, speed);
    digitalWrite(LEFT_MOTOR_IN1, LOW);
    digitalWrite(LEFT_MOTOR_IN2, HIGH);
    digitalWrite(RIGHT_MOTOR_IN1, LOW);
    digitalWrite(RIGHT_MOTOR_IN2, HIGH);
  }
}

void turnLeft(int speed) {
  if (!isKilled) {
    analogWrite(LEFT_MOTOR_EN, speed);
    analogWrite(RIGHT_MOTOR_EN, speed);
    digitalWrite(LEFT_MOTOR_IN1, LOW);
    digitalWrite(LEFT_MOTOR_IN2, HIGH);
    digitalWrite(RIGHT_MOTOR_IN1, HIGH);
    digitalWrite(RIGHT_MOTOR_IN2, LOW);
  }
}

void turnRight(int speed) {
  if (!isKilled) {
    analogWrite(LEFT_MOTOR_EN, speed);
    analogWrite(RIGHT_MOTOR_EN, speed);
    digitalWrite(LEFT_MOTOR_IN1, HIGH);
    digitalWrite(LEFT_MOTOR_IN2, LOW);
    digitalWrite(RIGHT_MOTOR_IN1, LOW);
    digitalWrite(RIGHT_MOTOR_IN2, HIGH);
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

// Test functions
void basicMovementTest() {
  Serial.println("Starting basic movement test...");
  
  // Forward
  Serial.println("Moving forward...");
  moveForward(200);
  delay(2000);
  
  // Backward
  Serial.println("Moving backward...");
  moveBackward(200);
  delay(2000);
  
  // Left turn
  Serial.println("Turning left...");
  turnLeft(200);
  delay(1000);
  
  // Right turn
  Serial.println("Turning right...");
  turnRight(200);
  delay(1000);
  
  stopMotors();
  Serial.println("Basic movement test complete");
}

void squarePathTest(bool clockwise) {
  Serial.println(clockwise ? "Starting clockwise square path test..." : "Starting counter-clockwise square path test...");
  
  for (int i = 0; i < 4; i++) {
    // Move forward one side
    Serial.println("Moving forward...");
    moveForward(200);
    waitForEncoderCount(SQUARE_SIDE);
    
    // Turn 90 degrees
    Serial.println("Turning...");
    if (clockwise) {
      turnRight(200);
    } else {
      turnLeft(200);
    }
    waitForEncoderCount(TURN_COUNTS);
    
    delay(500); // Brief pause between movements
  }
  
  stopMotors();
  Serial.println("Square path test complete");
}

void speedControlTest() {
  Serial.println("Starting speed control test...");
  
  // Gradually increase speed
  for (int speed = 0; speed <= MAX_SPEED; speed += 10) {
    Serial.print("Speed: ");
    Serial.println(speed);
    moveForward(speed);
    delay(500);
  }
  
  // Gradually decrease speed
  for (int speed = MAX_SPEED; speed >= 0; speed -= 10) {
    Serial.print("Speed: ");
    Serial.println(speed);
    moveForward(speed);
    delay(500);
  }
  
  stopMotors();
  Serial.println("Speed control test complete");
}

// Helper functions
void waitForEncoderCount(int targetCount) {
  long startTime = millis();
  while (abs(leftEncoderCount) < targetCount && abs(rightEncoderCount) < targetCount) {
    if (millis() - startTime > 5000) { // 5-second timeout
      Serial.println("Timeout waiting for encoder count");
      break;
    }
  }
  stopMotors();
}

void printEncoderCounts() {
  Serial.print("Left Encoder: ");
  Serial.print(leftEncoderCount);
  Serial.print(" Right Encoder: ");
  Serial.println(rightEncoderCount);
}

void calculateAndPrintRPM() {
  long leftCount = leftEncoderCount - lastLeftCount;
  long rightCount = rightEncoderCount - lastRightCount;

  // Calculate RPM
  float leftRPM = (leftCount / (float)ENCODER_PULSES_PER_REV) * 60.0;
  float rightRPM = (rightCount / (float)ENCODER_PULSES_PER_REV) * 60.0;

  // Update last counts
  lastLeftCount = leftEncoderCount;
  lastRightCount = rightEncoderCount;

  // Print RPM
  Serial.print("Left RPM: ");
  Serial.print(leftRPM);
  Serial.print(" | Right RPM: ");
  Serial.println(rightRPM);
} 
