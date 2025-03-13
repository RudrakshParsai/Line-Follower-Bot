// IR Sensor Array Test Program
// Pins for 5 IR sensors (adjust these pins based on your connections)
const int IR_LEFT_2 = 2;    // Leftmost sensor
const int IR_LEFT_1 = 3;    // Left sensor
const int IR_CENTER = 4;    // Center sensor
const int IR_RIGHT_1 = 5;   // Right sensor
const int IR_RIGHT_2 = 6;   // Rightmost sensor

// Sensor states
int sensorStates[5] = {0, 0, 0, 0, 0};

void setup() {
  Serial.begin(9600);
  
  // Configure IR sensor pins as inputs with pull-up resistors
  pinMode(IR_LEFT_2, INPUT_PULLUP);
  pinMode(IR_LEFT_1, INPUT_PULLUP);
  pinMode(IR_CENTER, INPUT_PULLUP);
  pinMode(IR_RIGHT_1, INPUT_PULLUP);
  pinMode(IR_RIGHT_2, INPUT_PULLUP);
  
  Serial.println("IR Sensor Array Test Program");
  Serial.println("----------------------------");
}

void loop() {
  // Read all sensors
  sensorStates[0] = digitalRead(IR_LEFT_2);
  sensorStates[1] = digitalRead(IR_LEFT_1);
  sensorStates[2] = digitalRead(IR_CENTER);
  sensorStates[3] = digitalRead(IR_RIGHT_1);
  sensorStates[4] = digitalRead(IR_RIGHT_2);
  
  // Print sensor states
  Serial.print("Sensor States: ");
  for(int i = 0; i < 5; i++) {
    Serial.print(sensorStates[i]);
    Serial.print(" ");
  }
  Serial.println();
  
  // Determine robot action based on sensor readings
  String command = determineCommand();
  Serial.print("Command: ");
  Serial.println(command);
  
  delay(500); // Delay for readability
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
    return "STOP - All sensors on line";
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