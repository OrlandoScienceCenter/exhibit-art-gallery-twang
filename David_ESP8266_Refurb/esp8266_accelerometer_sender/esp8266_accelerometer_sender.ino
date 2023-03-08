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

#include <BMI160Gen.h>
 
// Accel info
const int select_pin = 10;
const int i2c_addr = 0x69;

// REPLACE WITH RECEIVER MAC Address
uint8_t broadcastAddress[] = {0xA0, 0x20, 0xA6, 0x18, 0x4E, 0x3F};

// Structure example to send data
// Must match the receiver structure
typedef struct struct_message 
{
  int gx;
  int gy;
  int gz;

  int ax; 
  int ay;
  int az;

  bool buttonPressed = false;
} 
struct_message;

// Create a struct_message called myData
struct_message myData;

unsigned long lastTime = 0;  
unsigned long timerDelay = 100;  // send readings timer

// Callback when data is sent
void OnDataSent(uint8_t *mac_addr, uint8_t sendStatus) {
  Serial.print("Last Packet Send Status: ");
  if (sendStatus == 0){
    Serial.println("Delivery success");
  }
  else{
    Serial.println("Delivery fail");
  }
}
 
void setup() {
  // Init Serial Monitor
  Serial.begin(115200);
 
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  pinMode(D4, INPUT_PULLUP);

  // Init ESP-NOW
  if (esp_now_init() != 0) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  esp_now_set_self_role(ESP_NOW_ROLE_CONTROLLER);
  esp_now_register_send_cb(OnDataSent);
  
  // Register peer
  esp_now_add_peer(broadcastAddress, ESP_NOW_ROLE_SLAVE, 1, NULL, 0);

  // Accel init
  BMI160.begin(BMI160GenClass::I2C_MODE, i2c_addr);
}
 
void loop() {
  if ((millis() - lastTime) > timerDelay) {

    int gx, gy, gz;         // raw gyro values
    int ax, ay, az;         // raw accel values
  
    // read raw gyro measurements from device
    BMI160.readGyro(gx, gy, gz);
    BMI160.readAccelerometer(ax, ay, az);
    
    myData.gx = gx;
    myData.gy = gy;
    myData.gz = gz;

    myData.ax = ax;
    myData.ay = ay;
    myData.az = az;

    myData.buttonPressed = !digitalRead(D4);

    // Send message via ESP-NOW
    esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));

    lastTime = millis();
  }
}