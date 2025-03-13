LINE FOLLOWER ROBOT - PIN CONNECTIONS AND COMPONENTS
=================================================

COMPONENTS REQUIRED:
------------------
1. Arduino Uno/Nano
2. 5x IR Line Following Sensors (TCRT5000)
3. 1x HC-SR04 Ultrasonic Sensor
4. 2x DC Motors (12V)
5. L298N Motor Driver Module
6. I2C LCD Display (16x2)
7. Jumper Wires
8. Power Supply (12V Battery)
9. Chassis and Wheels
10. Emergency Kill Switch (Push Button)

PIN CONNECTIONS:
---------------

1. IR SENSORS:
   - IR_LEFT_2  -> Arduino Pin 2
   - IR_LEFT_1  -> Arduino Pin 3
   - IR_CENTER  -> Arduino Pin 4
   - IR_RIGHT_1 -> Arduino Pin 5
   - IR_RIGHT_2 -> Arduino Pin 6
   - VCC        -> 5V
   - GND        -> GND

2. ULTRASONIC SENSOR (HC-SR04):
   - TRIG_PIN   -> Arduino Pin 7
   - ECHO_PIN   -> Arduino Pin 8
   - VCC        -> 5V
   - GND        -> GND

3. MOTOR DRIVER (L298N):
   - LEFT_MOTOR_IN1  -> Arduino Pin 9
   - LEFT_MOTOR_IN2  -> Arduino Pin 10
   - RIGHT_MOTOR_IN1 -> Arduino Pin 11
   - RIGHT_MOTOR_IN2 -> Arduino Pin 12
   - LEFT_MOTOR_EN   -> Arduino Pin 13 (PWM)
   - RIGHT_MOTOR_EN  -> Arduino Pin 14 (PWM)
   - 12V             -> 12V Battery Positive
   - GND             -> GND
   - 5V              -> 5V (Optional, if not using external power)

4. I2C LCD DISPLAY:
   - SDA            -> Arduino A4
   - SCL            -> Arduino A5
   - VCC            -> 5V
   - GND            -> GND

5. EMERGENCY KILL SWITCH:
   - One terminal -> Arduino Pin 15 (or any available digital pin)
   - Other terminal -> GND
   - Mount on top of robot for easy access

POWER CONNECTIONS:
-----------------
1. 12V Battery:
   - Positive -> L298N 12V
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
   - Connect one terminal to an available digital pin (e.g., Pin 15)
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
5. Motors should be connected to the appropriate output terminals on the L298N
6. Double-check all connections before powering on
7. Use appropriate wire gauges for power connections
8. Always test the kill switch before starting the robot

TROUBLESHOOTING:
---------------
1. If LCD doesn't work:
   - Check I2C address (use I2C scanner sketch)
   - Verify SDA and SCL connections
   - Ensure proper power supply

2. If motors don't move:
   - Check motor driver connections
   - Verify PWM pins are correctly connected
   - Check power supply voltage

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