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
 
// Uncomment this to enable serial and debug printouts
#define DEBUG_MODE_ON

// Uncomment this to enable printouts about packet send status with ESP-NOW
#define DEBUG_NETWORK_ON

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

// Create a struct_message called accelGyroData
struct_message accelGyroData;

unsigned long lastTime = 0;  
unsigned long timerDelay = 100;  // send readings timer

// Callback when data is sent
void OnDataSent(uint8_t *mac_addr, uint8_t sendStatus) {
  
  #ifdef DEBUG_NETWORK_ON
  
    Serial.print(F("Last Packet Send Status: "));
    if (sendStatus == 0)
    {
      Serial.println(F("Delivery success"));
    }
    else
    {
      Serial.println(F("Delivery fail"));
    }

    Serial.println();
    Serial.println();

  #endif

}
 
void setup() 
{
  // Init Serial Monitor
  Serial.begin(115200);

  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  pinMode(D4, INPUT_PULLUP);

  // Init ESP-NOW
  if (esp_now_init() != 0) 
  {
    #ifdef DEBUG_MODE_ON

      Serial.println("Error initializing ESP-NOW");
    
    #endif

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
 
void loop() 
{
  if ((millis() - lastTime) > timerDelay) 
  {  
    loadAccelGyroData();

    // Uncomment this to erase what's in accelGyroData and replace it with mock values for testing
    //loadMockAccelGyroData();

    modifyAccelGyroData();

    sendAccelGyroDataOverEspNow();

    #ifdef DEBUG_MODE_ON

      debugAccelerometerValuesReadout();

    #endif

    lastTime = millis();
  }
}

void loadAccelGyroData()
{
  // read raw gyro measurements from device
  BMI160.readGyro(accelGyroData.gx, accelGyroData.gy, accelGyroData.gz);

  BMI160.readAccelerometer(accelGyroData.ax, accelGyroData.ay, accelGyroData.az);
  
  accelGyroData.buttonPressed = !digitalRead(D4);
}

void loadMockAccelGyroData()
{
  accelGyroData.ax = 0;
  accelGyroData.ay = 0;
  accelGyroData.az = 0;

  accelGyroData.gx = 0;
  accelGyroData.gy = 0;
  accelGyroData.gz = 0;
  
  accelGyroData.buttonPressed = false;

  // accelGyroData.ax = 0;
  // accelGyroData.ay = 0;
  // accelGyroData.az = 0;

  // accelGyroData.gx = 0;
  // accelGyroData.gy = 0;
  // accelGyroData.gz = 0;
}

void modifyAccelGyroData()
{
  accelGyroData.ax = map(accelGyroData.ax, -17000, 17000, 0, 50);
  accelGyroData.ay = map(accelGyroData.ay, -17000, 17000, 50, 0);
  accelGyroData.az = map(accelGyroData.az, -17000, 17000, 0, 50);

  // These will obviously override the above rows
  // accelGyroData.ax = 0;
  // accelGyroData.ay = 0;
  // accelGyroData.az = 0;
  
  accelGyroData.gx = map(accelGyroData.gx, -17000, 17000, 0, 50);
  accelGyroData.gy = map(accelGyroData.gy, -17000, 17000, 0, 50);
  accelGyroData.gz = map(accelGyroData.gz, -17000, 17000, 0, 50);

  // These will obviously override the above rows
  // accelGyroData.gx = 0;
  // accelGyroData.gy = 0;
  // accelGyroData.gz = 0;
}

void sendAccelGyroDataOverEspNow()
{
  // Send message via ESP-NOW
  esp_now_send(broadcastAddress, (uint8_t *) &accelGyroData, sizeof(accelGyroData));
}

void debugAccelerometerValuesReadout()
{
  #ifdef DEBUG_MODE_ON

    // Accel readouts
    Serial.print(F("aX: "));
    Serial.print(accelGyroData.ax);
    Serial.print(F(", "));

    Serial.print(F("aY: "));
    Serial.print(accelGyroData.ay);
    Serial.print(F(", "));

    Serial.print(F("aZ: "));
    Serial.println(accelGyroData.az);

    // Gyro readouts
    Serial.print(F("gX: "));
    Serial.print(accelGyroData.gx);
    Serial.print(F(", "));

    Serial.print(F("gY: "));
    Serial.print(accelGyroData.gy);
    Serial.print(F(", "));

    Serial.print(F("gZ: "));
    Serial.println(accelGyroData.gz);

    // Button readout
    
    Serial.print(F("Button: "));
    Serial.println(accelGyroData.buttonPressed);
    
    Serial.println();
    Serial.println();

  #endif
}