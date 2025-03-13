// Ultrasonic Sensor Test Program with IR Integration
// Pins for 5 IR sensors (same as IR test program)
const int IR_LEFT_2 = 2;    // Leftmost sensor
const int IR_LEFT_1 = 3;    // Left sensor
const int IR_CENTER = 4;    // Center sensor
const int IR_RIGHT_1 = 5;   // Right sensor
const int IR_RIGHT_2 = 6;   // Rightmost sensor

// HC-SR04 Ultrasonic Sensor Pins
const int TRIG_PIN = 7;
const int ECHO_PIN = 8;

// Constants for ultrasonic sensor
const int WALL_DISTANCE = 20;  // Distance in cm to detect wall
const int MAX_DISTANCE = 400;  // Maximum distance to measure
const int MIN_DISTANCE = 2;    // Minimum distance to measure

// Variables for ultrasonic sensor
long duration;
int distance;
bool wallDetected = false;

// Sensor states for IR
int sensorStates[5] = {0, 0, 0, 0, 0};

void setup() {
  Serial.begin(9600);
  
  // Configure IR sensor pins
  pinMode(IR_LEFT_2, INPUT_PULLUP);
  pinMode(IR_LEFT_1, INPUT_PULLUP);
  pinMode(IR_CENTER, INPUT_PULLUP);
  pinMode(IR_RIGHT_1, INPUT_PULLUP);
  pinMode(IR_RIGHT_2, INPUT_PULLUP);
  
  // Configure ultrasonic sensor pins
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  
  Serial.println("Ultrasonic Sensor Test with IR Integration");
  Serial.println("----------------------------------------");
}

void loop() {
  // Read IR sensors
  readIRSensors();
  
  // Read ultrasonic sensor
  readUltrasonicSensor();
  
  // Print sensor readings
  printSensorData();
  
  // Check for wall and determine action
  checkWallAndDetermineAction();
  
  delay(100); // Small delay between readings
}

void readIRSensors() {
  sensorStates[0] = digitalRead(IR_LEFT_2);
  sensorStates[1] = digitalRead(IR_LEFT_1);
  sensorStates[2] = digitalRead(IR_CENTER);
  sensorStates[3] = digitalRead(IR_RIGHT_1);
  sensorStates[4] = digitalRead(IR_RIGHT_2);
}

void readUltrasonicSensor() {
  // Clear trigger pin
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  
  // Set trigger pin high for 10 microseconds
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  
  // Read echo pin, return travel time in microseconds
  duration = pulseIn(ECHO_PIN, HIGH);
  
  // Calculate distance in cm
  distance = duration * 0.034 / 2;
  
  // Validate distance reading
  if (distance > MAX_DISTANCE || distance < MIN_DISTANCE) {
    distance = -1; // Invalid reading
  }
  
  // Update wall detection status
  wallDetected = (distance <= WALL_DISTANCE && distance > 0);
}

void printSensorData() {
  // Print IR sensor states
  Serial.print("IR Sensors: ");
  for(int i = 0; i < 5; i++) {
    Serial.print(sensorStates[i]);
    Serial.print(" ");
  }
  
  // Print ultrasonic sensor data
  Serial.print(" | Distance: ");
  if (distance == -1) {
    Serial.print("Invalid");
  } else {
    Serial.print(distance);
    Serial.print(" cm");
  }
  
  Serial.print(" | Wall Detected: ");
  Serial.println(wallDetected ? "YES" : "NO");
}

void checkWallAndDetermineAction() {
  if (wallDetected) {
    // Wall detected - determine if we should stop or U-turn
    bool onLine = false;
    for(int i = 0; i < 5; i++) {
      if (sensorStates[i] == 1) {
        onLine = true;
        break;
      }
    }
    
    if (onLine) {
      Serial.println("COMMAND: U_TURN - Wall detected while on line");
    } else {
      Serial.println("COMMAND: STOP - Wall detected while off line");
    }
  } else {
    // No wall detected, continue normal line following
    String command = determineCommand();
    Serial.print("COMMAND: ");
    Serial.println(command);
  }
}

String determineCommand() {
  // All sensors off line
  if (sensorStates[0] == 0 && sensorStates[1] == 0 && sensorStates[2] == 0 && 
      sensorStates[3] == 0 && sensorStates[4] == 0) {
    return "STOP - No line detected";
  }
  
  // All sensors on line
  if (sensorStates[0] == 1 && sensorStates[1] == 1 && sensorStates[2] == 1 && 
      sensorStates[3] == 1 && sensorStates[4] == 1) {
    return "FORWARD - All sensors on line";
  }
  
  // Perfect alignment (center sensor on line)
  if (sensorStates[2] == 1 && sensorStates[1] == 0 && sensorStates[3] == 0) {
    return "FORWARD - Perfect alignment";
  }
  
  // Slight left adjustment needed
  if (sensorStates[1] == 1 && sensorStates[2] == 0) {
    return "SLIGHT_LEFT - Adjust left";
  }
  
  // Slight right adjustment needed
  if (sensorStates[3] == 1 && sensorStates[2] == 0) {
    return "SLIGHT_RIGHT - Adjust right";
  }
  
  // Sharp left turn needed
  if (sensorStates[0] == 1 || (sensorStates[0] == 1 && sensorStates[1] == 1)) {
    return "SHARP_LEFT - Turn left";
  }
  
  // Sharp right turn needed
  if (sensorStates[4] == 1 || (sensorStates[4] == 1 && sensorStates[3] == 1)) {
    return "SHARP_RIGHT - Turn right";
  }
  
  // Lost line - need to search
  if (sensorStates[0] == 0 && sensorStates[1] == 0 && sensorStates[2] == 0 && 
      (sensorStates[3] == 1 || sensorStates[4] == 1)) {
    return "SEARCH_RIGHT - Lost line, searching right";
  }
  
  if (sensorStates[0] == 0 && sensorStates[1] == 0 && sensorStates[2] == 0 && 
      (sensorStates[0] == 1 || sensorStates[1] == 1)) {
    return "SEARCH_LEFT - Lost line, searching left";
  }
  
  return "UNKNOWN - Unexpected sensor pattern";
} 