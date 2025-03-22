#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_ADDR 0x3C 
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire);

#define NODE_SENSOR_A 2
#define NODE_SENSOR_B 3
#define NODE_SENSOR_C 3
#define NODE_SENSOR_D 4

int nodeCounter = 0;
int maxSpeed = 150; 

void setup() {
    Wire.begin();
    
    if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
        Serial.println(F("SSD1306 allocation failed"));
        for (;;);
    }
    
    display.clearDisplay();
    display.setTextSize(2);      
    display.setTextColor(WHITE); 
    display.display();  

    pinMode(NODE_SENSOR_A, INPUT);
    pinMode(NODE_SENSOR_B, INPUT);
    pinMode(NODE_SENSOR_C, INPUT);
    pinMode(NODE_SENSOR_D, INPUT);
}

void readNode(){
    int A = digitalRead(NODE_SENSOR_A);
    int B = digitalRead(NODE_SENSOR_B);
    int C = digitalRead(NODE_SENSOR_C);
    int D = digitalRead(NODE_SENSOR_D);

    int nodeValue = (A * 8) + (B * 4) + (C * 2) + D;
    return nodeValue;
}
void detectNode(){
    
    int nodeValue = readNode();
    nodeCounter++;
    if (nodeCounter == 1) {
        maxSpeed = (nodeValue / 5.0) * 255;
    } else if (nodeCounter == 2 && nodeValue == 0) {
        maxSpeed = 255;
    } else if (nodeCounter == 3) {
        // angle = theta * 10 
    }

    display.clearDisplay();
    display.setCursor(10, 10);
    display.print("Node detected!");
    
    display.setCursor(10, 20);
    display.setTextSize(2);
    display.print("Node Value: ");
    display.print(nodeValue);

    display.setCursor(10, 40);
    display.print("Node Count: ");
    display.print(nodeCounter);

    display.display();
    delay(100);
}
void loop() {
   
}
