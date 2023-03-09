#include <ESP8266WiFi.h>
#include <espnow.h>

#include "RunningMedian.h"
#include <SoftwareSerial.h>

#define DEBUG_MODE_ON                   // Uncomment this line if you want to see debug messages

#define MYPORT_TX D5
#define MYPORT_RX D6

SoftwareSerial teenseyPort;

typedef struct structEspNowPacket                                           // Structure example to send data, must match the sender structure
{
  int joystickAveragedTilt;
  int joystickPeakWobble;

  bool buttonPressed = false;
} 
structEspNowPacket;

structEspNowPacket pedestalData;                               // Create a structEspNowPacket called pedestalData to deserialize the incoming packet from the sender over esp-now

bool ledState = false;

#ifdef DEBUG_MODE_ON

  unsigned long lastTime = 0;  
  unsigned long timerDelay = 1000;                             // send readings timer

#endif 

void OnDataRecv(uint8_t * mac, uint8_t *incomingData, uint8_t len)      // Callback function that will be executed when data is received
{
  memcpy(&pedestalData, incomingData, sizeof(pedestalData));

  toggleBuiltInLed();

  sendJoystickValuesToTeensey();
}
 
void setup() 
{
  Serial.begin(115200);

  pinMode(LED_BUILTIN, OUTPUT);

  initializeSoftwareSerialPortToTeensey();

  initializeEspNow();
}

void loop() 
{
  #ifdef DEBUG_MODE_ON
  
    if ((millis() - lastTime) > timerDelay) 
    {  
      debugPrintToSerialJoystickValues();
    }

    lastTime = millis();

  #endif
}

void debugPrintToSerialJoystickValues()
{
  Serial.println();
  Serial.println();

  // Serial.print(F("Bytes received: "));
  // Serial.println(len);

  Serial.print(F("Tilt from pedestal: "));
  Serial.print(pedestalData.joystickAveragedTilt);

  Serial.print(F(", peak recent wobble value from pedestal: "));
  Serial.print(pedestalData.joystickPeakWobble);
  
  Serial.print(F(", button state: "));
  Serial.println(pedestalData.buttonPressed);
}

void sendJoystickValuesToTeensey()
{
  teenseyPort.print(F("t"));
  teenseyPort.print(pedestalData.joystickAveragedTilt);
  teenseyPort.print(F(";"));

  teenseyPort.print(F("w"));
  teenseyPort.print(pedestalData.joystickPeakWobble);
  teenseyPort.print(F(","));

  teenseyPort.print(F("b"));
  teenseyPort.print(pedestalData.buttonPressed);
  teenseyPort.println(F("."));
}

void toggleBuiltInLed()
{
  digitalWrite(LED_BUILTIN, ledState);
  
  ledState = !ledState;
}

void initializeSoftwareSerialPortToTeensey()
{
  teenseyPort.begin(9600, SWSERIAL_8N1, MYPORT_RX, MYPORT_TX, false);

  if (!teenseyPort) 
  { 
    // If the object did not initialize, then its configuration is invalid
    Serial.println(F("Invalid SoftwareSerial pin configuration, check config")); 
    
    // Don't continue with invalid configuration
    while (1) { delay (1000); }
  } 
}

void initializeEspNow()
{
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != 0) {
    Serial.println(F("Error initializing ESP-NOW"));
    return;
  }
  
  // Once ESPNow is successfully Init, register for recieve callback to get recieved packet
  esp_now_set_self_role(ESP_NOW_ROLE_SLAVE);
  esp_now_register_recv_cb(OnDataRecv);
}