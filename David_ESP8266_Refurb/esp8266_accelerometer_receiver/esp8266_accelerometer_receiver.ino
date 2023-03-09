#include <ESP8266WiFi.h>
#include <espnow.h>

#include "RunningMedian.h"
#include <SoftwareSerial.h>

#define DEBUG_MODE_ON                   // Uncomment this line if you want to see debug messages

#define MYPORT_TX D5
#define MYPORT_RX D6

// JOYSTICK
#define JOYSTICK_ORIENTATION 1          // 0, 1 or 2 to set the angle of the joystick
#define JOYSTICK_DIRECTION   1          // 0/1 to flip joystick direction
#define ATTACK_THRESHOLD     30000      // The threshold that triggers an attack
#define JOYSTICK_DEADZONE    5          // Angle to ignore

SoftwareSerial teenseyPort;

typedef struct struct_espnow_packet     // Structure example to receive data, must match the sender structure
{
  int gx;
  int gy;
  int gz;

  int ax; 
  int ay;
  int az;

  bool buttonPressed;
} 

struct_espnow_packet;

int joystickTilt = 0;                   // Stores the angle of the joystick
int joystickWobble = 0;                 // Stores the max amount of acceleration (wobble)

bool ledState = false;

RunningMedian MPUAngleSamples = RunningMedian(2);
RunningMedian MPUWobbleSamples = RunningMedian(2);

struct_espnow_packet accelGyroData;     // Create a struct_espnow_packet called accelGyroData for esp-now

void OnDataRecv(uint8_t * mac, uint8_t *incomingData, uint8_t len)      // Callback function that will be executed when data is received
{
  handleIncomingEspNowPacket();
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
  mockJoystickInputToAccelGyroData();

  modifyAccelGyroData();  // Do the maps here. Later, this can be moved to sender code

  getWobblerJoystickInputFromAccelGyroData();

  #ifdef DEBUG_MODE_ON

    debugPrintToSerialJoystickValues();

  #endif

}

void handleIncomingEspNowPacket()
{
  toggleBuiltInLed();

  memcpy(&accelGyroData, incomingData, sizeof(accelGyroData));

  getWobblerJoystickInputFromAccelGyroData();

  sendJoystickValuesToTeensey();
}

void mockJoystickInputToAccelGyroData()
{
  // Vertical, untouched
  accelGyroData.ax = 6241;
  accelGyroData.ay = -15578;
  accelGyroData.az = 707;

  accelGyroData.gx = 6251;
  accelGyroData.gy = -15567;
  accelGyroData.gz = 740;

  // bent left 90, should make dot go forwards
  // accelGyroData.ax = -14657;
  // accelGyroData.ay = -3578;
  // accelGyroData.az = 5921;

  // accelGyroData.gx = -62;
  // accelGyroData.gy = 139;
  // accelGyroData.gz = -352;

  // bent right 90, should make dot go backwards
  // accelGyroData.ax = 14775;
  // accelGyroData.ay = 5880;
  // accelGyroData.az = -4572;

  // accelGyroData.gx = 14775;
  // accelGyroData.gy = 5880;
  // accelGyroData.gz = -4572;  
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

void getWobblerJoystickInputFromAccelGyroData()
{
  // This is responsible for the player movement speed and attacking. 
  // You can replace it with anything you want that passes a -90>+90 value to joystickTilt
  // and any value to joystickWobble that is greater than ATTACK_THRESHOLD (defined at start)
  // For example you could use 3 momentery buttons:
  // if(digitalRead(leftButtonPinNumber) == HIGH) joystickTilt = -90;
  // if(digitalRead(rightButtonPinNumber) == HIGH) joystickTilt = 90;
  // if(digitalRead(attackButtonPinNumber) == HIGH) joystickWobble = ATTACK_THRESHOLD;
  
  int a = (JOYSTICK_ORIENTATION == 0?accelGyroData.ax:(JOYSTICK_ORIENTATION == 1?accelGyroData.ay:accelGyroData.az))/166;
  int g = (JOYSTICK_ORIENTATION == 0?accelGyroData.gx:(JOYSTICK_ORIENTATION == 1?accelGyroData.gy:accelGyroData.gz));

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

void debugPrintToSerialJoystickValues()
{
  // Serial.print(F("Bytes received: "));
  // Serial.println(len);

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


  Serial.print(F("t"));
  Serial.print(joystickTilt);
  Serial.print(F(";"));

  Serial.print(F("w"));
  Serial.print(joystickWobble);
  Serial.print(F(","));

  Serial.print(F("b"));
  Serial.print(accelGyroData.buttonPressed);
  Serial.println("."));
}

void sendJoystickValuesToTeensey()
{
  teenseyPort.print("t"));
  teenseyPort.print(joystickTilt);
  teenseyPort.print(";"));

  teenseyPort.print("w"));
  teenseyPort.print(joystickWobble);
  teenseyPort.print(","));

  teenseyPort.print("b"));
  teenseyPort.print(accelGyroData.buttonPressed);
  teenseyPort.println("."));
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
    Serial.println("Invalid SoftwareSerial pin configuration, check config")); 
    
    // Don't continue with invalid configuration
    while (1) { delay (1000); }
  } 
}

void initializeEspNow()
{
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != 0) {
    Serial.println("Error initializing ESP-NOW"));
    return;
  }
  
  // Once ESPNow is successfully Init, register for recieve callback to get recieved packet
  esp_now_set_self_role(ESP_NOW_ROLE_SLAVE);
  esp_now_register_recv_cb(OnDataRecv);
}