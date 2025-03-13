LINE FOLLOWER ROBOT - PIN CONNECTIONS AND COMPONENTS
=================================================

COMPONENTS REQUIRED:
------------------
1. Arduino Mega 2560
2. 5x IR Line Following Sensors (TCRT5000) - Main Array
3. 4x IR Line Following Sensors (TCRT5000) - Additional Sensors
4. 1x HC-SR04 Ultrasonic Sensor
5. 2x DC Motors (12V)
6. TB6612FNG Motor Driver Module
7. I2C LCD Display (16x2)
8. Jumper Wires
9. Power Supply (12V Battery)
10. Chassis and Wheels
11. Emergency Kill Switch (Push Button)

PIN CONNECTIONS:
---------------

1. MAIN IR SENSOR ARRAY (5 sensors):
   - IR_LEFT_2  -> Arduino Pin 2
   - IR_LEFT_1  -> Arduino Pin 3
   - IR_CENTER  -> Arduino Pin 4
   - IR_RIGHT_1 -> Arduino Pin 5
   - IR_RIGHT_2 -> Arduino Pin 6
   - VCC        -> 5V
   - GND        -> GND

2. ADDITIONAL IR SENSORS (4 sensors):
   - IR_ADD_1   -> Arduino Pin 22
   - IR_ADD_2   -> Arduino Pin 23
   - IR_ADD_3   -> Arduino Pin 24
   - IR_ADD_4   -> Arduino Pin 25
   - VCC        -> 5V
   - GND        -> GND

3. ULTRASONIC SENSOR (HC-SR04):
   - TRIG_PIN   -> Arduino Pin 7
   - ECHO_PIN   -> Arduino Pin 8
   - VCC        -> 5V
   - GND        -> GND

4. MOTOR DRIVER (TB6612FNG):
   - PWMA       -> Arduino Pin 9 (PWM)
   - AIN1       -> Arduino Pin 10
   - AIN2       -> Arduino Pin 11
   - PWMB       -> Arduino Pin 12 (PWM)
   - BIN1       -> Arduino Pin 13
   - BIN2       -> Arduino Pin 14
   - STBY       -> Arduino Pin 15 (Standby control)
   - VM         -> 12V Battery Positive
   - VCC        -> 5V
   - GND        -> GND

5. I2C LCD DISPLAY:
   - SDA        -> Arduino Pin 20 (Mega's SDA)
   - SCL        -> Arduino Pin 21 (Mega's SCL)
   - VCC        -> 5V
   - GND        -> GND

6. EMERGENCY KILL SWITCH:
   - One terminal -> Arduino Pin 16 (or any available digital pin)
   - Other terminal -> GND
   - Mount on top of robot for easy access

POWER CONNECTIONS:
-----------------
1. 12V Battery:
   - Positive -> TB6612FNG VM
   - Negative -> GND

2. Arduino Power:
   - VIN      -> 12V Battery Positive (if using external power)
   - GND      -> GND

KILL SWITCH IMPLEMENTATION:
-------------------------
1. Physical Placement:
   - Mount the kill switch on top of the robot chassis
   - Ensure it's easily accessible in case of emergency
   - Use a large, brightly colored button for visibility
   - Consider adding a protective cover to prevent accidental activation

2. Wiring:
   - Connect one terminal to an available digital pin (e.g., Pin 16)
   - Connect the other terminal to GND
   - Use a pull-up resistor (10kΩ) between the pin and 5V
   - Keep wires short and secure to prevent disconnection

3. Operation:
   - When pressed, the kill switch will immediately stop all motors
   - The robot will remain in a safe state until reset
   - LCD will display "EMERGENCY STOP" message
   - All sensor readings will continue to be displayed

4. Testing:
   - Test the kill switch before each run
   - Verify that motors stop immediately when pressed
   - Check that the robot remains in a safe state
   - Ensure the switch is not too sensitive to accidental bumps

NOTES:
-----
1. Make sure all GND connections are common
2. The I2C LCD address is 0x27 (can be changed if needed)
3. IR sensors should be mounted at appropriate height (typically 1-2cm from ground)
4. Ultrasonic sensor should be mounted facing forward
5. Motors should be connected to the appropriate output terminals on the TB6612FNG
6. Double-check all connections before powering on
7. Use appropriate wire gauges for power connections
8. Always test the kill switch before starting the robot
9. Arduino Mega has more PWM pins available for motor control
10. TB6612FNG has better efficiency than L298N

TROUBLESHOOTING:
---------------
1. If LCD doesn't work:
   - Check I2C address (use I2C scanner sketch)
   - Verify SDA and SCL connections (Mega uses pins 20 and 21)
   - Ensure proper power supply

2. If motors don't move:
   - Check motor driver connections
   - Verify PWM pins are correctly connected
   - Check power supply voltage
   - Ensure STBY pin is HIGH for normal operation

3. If sensors don't work:
   - Verify sensor connections
   - Check power supply to sensors
   - Ensure proper mounting height

4. If ultrasonic sensor gives erratic readings:
   - Check for interference
   - Verify power supply stability
   - Ensure proper mounting angle

5. If kill switch doesn't work:
   - Check wiring connections
   - Verify pull-up resistor is properly connected
   - Test the switch continuity with a multimeter
   - Ensure the switch is not stuck in pressed position

SAFETY PRECAUTIONS:
------------------
1. Always disconnect power before making connections
2. Double-check polarity of all connections
3. Use appropriate wire insulation
4. Keep battery terminals covered when not in use
5. Monitor battery voltage during operation
6. Keep robot away from water and moisture
7. Test kill switch before each run
8. Keep kill switch easily accessible
9. Have a backup power disconnect method (e.g., battery disconnect)
10. Never leave the robot unattended while running

ADDITIONAL INFORMATION:
---------------------
1. MAIN IR SENSOR ARRAY (5 sensors):
   - Using 5 sensors for precise line detection
   - Leftmost sensor (IR_LEFT_2) for sharp turns
   - Left sensor (IR_LEFT_1) for normal line following
   - Center sensor (IR_CENTER) for line detection and U-turns
   - Right sensor (IR_RIGHT_1) for normal line following
   - Rightmost sensor (IR_RIGHT_2) for sharp turns
   - Mount sensors in a straight line with equal spacing
   - Recommended spacing: 1.5-2cm between sensors
   - Mount height: 1-2cm from ground

2. ADDITIONAL IR SENSORS (4 sensors):
   - Mounted at different positions for additional functionality
   - Connected to digital pins 22-25
   - Purpose and mounting position to be specified by user

3. TB6612FNG MOTOR DRIVER:
   - More efficient than L298N (lower heat generation)
   - Higher current rating (1.2A continuous, 3.2A peak)
   - Built-in protection circuits
   - Standby mode for power saving
   - PWM frequency: 25kHz
   - Operating voltage: 2.5V to 13.5V
   - Low voltage operation possible
   - Better motor control with PWM

4. ARDUINO MEGA SPECIFICATIONS:
   - 54 digital I/O pins (15 PWM)
   - 16 analog input pins
   - 4 hardware serial ports
   - 256KB flash memory
   - 8KB SRAM
   - 4KB EEPROM
   - I2C on pins 20 (SDA) and 21 (SCL)
   - Operating voltage: 5V
   - Input voltage: 7-12V recommended 
