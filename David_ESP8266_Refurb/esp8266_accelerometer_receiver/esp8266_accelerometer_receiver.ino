/*
  Rui Santos
  Complete project details at https://RandomNerdTutorials.com/esp-now-esp8266-nodemcu-arduino-ide/
  
  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files.
  
  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.
*/

#include <ESP8266WiFi.h>
#include <espnow.h>

#include "RunningMedian.h"
#include <SoftwareSerial.h>

#define MYPORT_TX D5
#define MYPORT_RX D6

SoftwareSerial teenseyPort;

// Structure example to receive data
// Must match the sender structure
typedef struct struct_message 
{
  int gx;
  int gy;
  int gz;

  int ax; 
  int ay;
  int az;

  bool buttonPressed;
} 
struct_message;

// JOYSTICK
#define JOYSTICK_ORIENTATION 1     // 0, 1 or 2 to set the angle of the joystick
#define JOYSTICK_DIRECTION   1     // 0/1 to flip joystick direction
#define ATTACK_THRESHOLD     30000 // The threshold that triggers an attack
#define JOYSTICK_DEADZONE    5     // Angle to ignore

int joystickTilt = 0;              // Stores the angle of the joystick
int joystickWobble = 0;            // Stores the max amount of acceleration (wobble)

bool ledState = false;

RunningMedian MPUAngleSamples = RunningMedian(5);
RunningMedian MPUWobbleSamples = RunningMedian(5);

// Create a struct_message called myData
struct_message myData;

// Callback function that will be executed when data is received
void OnDataRecv(uint8_t * mac, uint8_t *incomingData, uint8_t len) 
{
digitalWrite(LED_BUILTIN, ledState);
ledState = !ledState;

  memcpy(&myData, incomingData, sizeof(myData));

  // Serial.print("Bytes received: ");
  // Serial.println(len);

  // Serial.print("gx: ");
  // Serial.println(myData.gx);
  // Serial.print("gy: ");
  // Serial.println(myData.gy);
  // Serial.print("gz: ");
  // Serial.println(myData.gz);
  // Serial.println();

  // Serial.print("ax: ");
  // Serial.println(myData.ax);
  // Serial.print("ay: ");
  // Serial.println(myData.ay);
  // Serial.print("az: ");
  // Serial.println(myData.az);
  // Serial.println();

  // Serial.print("buttonPressed: ");
  // Serial.println(myData.buttonPressed);
  // Serial.println();

  getInput();

  Serial.print("t");
  Serial.print(joystickTilt);
  Serial.print(";");

  Serial.print("w");
  Serial.print(joystickWobble);
  Serial.print(",");

  Serial.print("b");
  Serial.print(myData.buttonPressed);
  Serial.println(".");

  
  teenseyPort.print("t");
  teenseyPort.print(joystickTilt);
  teenseyPort.print(";");

  teenseyPort.print("w");
  teenseyPort.print(joystickWobble);
  teenseyPort.print(",");

  teenseyPort.print("b");
  teenseyPort.print(myData.buttonPressed);
  teenseyPort.println(".");
}
 
void setup() 
{
  // Initialize Serial Monitor
  Serial.begin(115200);

  pinMode(LED_BUILTIN, OUTPUT);

  teenseyPort.begin(9600, SWSERIAL_8N1, MYPORT_RX, MYPORT_TX, false);

  if (!teenseyPort) { // If the object did not initialize, then its configuration is invalid
    Serial.println("Invalid SoftwareSerial pin configuration, check config"); 
    while (1) { // Don't continue with invalid configuration
      delay (1000);
    }
  } 

  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != 0) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  
  // Once ESPNow is successfully Init, we will register for recv CB to
  // get recv packer info
  esp_now_set_self_role(ESP_NOW_ROLE_SLAVE);
  esp_now_register_recv_cb(OnDataRecv);
}

void loop()
{
  
}


// ---------------------------------
// ----------- JOYSTICK ------------
// ---------------------------------
void getInput()
{
    // This is responsible for the player movement speed and attacking. 
    // You can replace it with anything you want that passes a -90>+90 value to joystickTilt
    // and any value to joystickWobble that is greater than ATTACK_THRESHOLD (defined at start)
    // For example you could use 3 momentery buttons:
    // if(digitalRead(leftButtonPinNumber) == HIGH) joystickTilt = -90;
    // if(digitalRead(rightButtonPinNumber) == HIGH) joystickTilt = 90;
    // if(digitalRead(attackButtonPinNumber) == HIGH) joystickWobble = ATTACK_THRESHOLD;
    
    // accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    int a = (JOYSTICK_ORIENTATION == 0?myData.ax:(JOYSTICK_ORIENTATION == 1?myData.ay:myData.az))/166;
    int g = (JOYSTICK_ORIENTATION == 0?myData.gx:(JOYSTICK_ORIENTATION == 1?myData.gy:myData.gz));
    if(abs(a) < JOYSTICK_DEADZONE) a = 0;
    if(a > 0) a -= JOYSTICK_DEADZONE;
    if(a < 0) a += JOYSTICK_DEADZONE;

    MPUAngleSamples.add(a);
    MPUWobbleSamples.add(g);
    
    joystickTilt = MPUAngleSamples.getMedian();
    
    if(JOYSTICK_DIRECTION == 1) 
    {
         joystickTilt = 0-joystickTilt;
    }
    
    joystickWobble = abs(MPUWobbleSamples.getHighest());
}
