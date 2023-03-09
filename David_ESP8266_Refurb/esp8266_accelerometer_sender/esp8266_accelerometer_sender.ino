#include <ESP8266WiFi.h>
#include <espnow.h>

#include "RunningMedian.h"
#include <BMI160Gen.h>
 
#define DEBUG_MODE_ON                                                   // Uncomment this to enable serial and debug printouts
#define DEBUG_NETWORK_ON                                                // Uncomment this to enable printouts about packet send status with ESP-NOW

#define PIN_RESET_BUTTON D4

// JOYSTICK
#define JOYSTICK_ORIENTATION 1          // 0, 1 or 2 to set the angle of the joystick
#define JOYSTICK_DIRECTION   1          // 0/1 to flip joystick direction
#define ATTACK_THRESHOLD     30000      // The threshold that triggers an attack
#define JOYSTICK_DEADZONE    5          // Angle to ignore

#define ACCELEROMETER_I2C_ADDRESS 0x69

RunningMedian MPUAngleSamples = RunningMedian(5);
RunningMedian MPUWobbleSamples = RunningMedian(5);

uint8_t broadcastAddress[] = {0xA0, 0x20, 0xA6, 0x18, 0x4E, 0x3F};      // RECEIVER MAC Address

typedef struct structImuValues 
{
  int gx;
  int gy;
  int gz;

  int ax; 
  int ay;
  int az;
} 
structImuValues;

typedef struct structEspNowPacket                                           // Structure example to send data, must match the receiver structure
{
  int joystickAveragedTilt;
  int joystickPeakWobble;

  bool buttonPressed = false;
} 
structEspNowPacket;

structEspNowPacket pedestalData;                               // Create a structEspNowPacket called pedestalData to deserialize the incoming packet from the sender over esp-now

unsigned long lastTime = 0;  
unsigned long timerDelay = 100;                             // send readings timer

void OnDataSent(uint8_t *mac_addr, uint8_t sendStatus)      // Callback when data is sent
{
  
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
  Serial.begin(115200);

  pinMode(PIN_RESET_BUTTON, INPUT_PULLUP);

  initializeEspNow();

  // Accel init
  BMI160.begin(BMI160GenClass::I2C_MODE, ACCELEROMETER_I2C_ADDRESS);
}
 
void loop() 
{
  structImuValues rawImuValues = loadDataFromImu();

  rawImuValues = loadMockDataInstead();                                   // IMPORTANT: Comment this line out if you don't want to use mock data

  structImuValues mappedImuValues = mapRawImuValues(rawImuValues);

  setWobblerJoystickInputFromPedestalData(mappedImuValues);

  if ((millis() - lastTime) > timerDelay) 
  {  
    sendPedestalDataOverEspNow();

    #ifdef DEBUG_MODE_ON

      debugAccelerometerValuesReadout(mappedImuValues);

    #endif

    lastTime = millis();
  }
}

structImuValues loadDataFromImu()
{
  structImuValues rawImuValues;

  // read raw gyro measurements from device
  BMI160.readGyro(rawImuValues.gx, rawImuValues.gy, rawImuValues.gz);

  BMI160.readAccelerometer(rawImuValues.ax, rawImuValues.ay, rawImuValues.az);

  return rawImuValues;
}

structImuValues mapRawImuValues(structImuValues rawImuValues)
{
  rawImuValues.ax = map(rawImuValues.ax, -17000, 17000, -90, 90);
  rawImuValues.ay = map(rawImuValues.ay, -17000, 17000, -90, 90);
  rawImuValues.az = map(rawImuValues.az, -17000, 17000, -90, 90);

  // These will obviously override the above rows
  // rawImuValues.ax = 0;
  // rawImuValues.ay = 0;
  // rawImuValues.az = 0;
  
  rawImuValues.gx = map(rawImuValues.gx, -17000, 17000, -90, 90);
  rawImuValues.gy = map(rawImuValues.gy, -17000, 17000, -90, 90);
  rawImuValues.gz = map(rawImuValues.gz, -17000, 17000, -90, 90);

  // These will obviously override the above rows
  // rawImuValues.gx = 0;
  // rawImuValues.gy = 0;
  // rawImuValues.gz = 0;

  return rawImuValues;
}

void setWobblerJoystickInputFromPedestalData(structImuValues mappedImuValues)
{
  // This is responsible for the player movement speed and attacking. 
  // You can replace it with anything you want that passes a -90>+90 value to joystickTilt
  // and any value to joystickWobble that is greater than ATTACK_THRESHOLD (defined at start)
  // For example you could use 3 momentery buttons:
  // if(digitalRead(leftButtonPinNumber) == HIGH) joystickTilt = -90;
  // if(digitalRead(rightButtonPinNumber) == HIGH) joystickTilt = 90;
  // if(digitalRead(attackButtonPinNumber) == HIGH) joystickWobble = ATTACK_THRESHOLD;
  
  // int a = (JOYSTICK_ORIENTATION == 0 ? pedestalData.ax : (JOYSTICK_ORIENTATION == 1 ? pedestalData.ay : pedestalData.az)) / 166;
  // int g = (JOYSTICK_ORIENTATION == 0 ? pedestalData.gx : (JOYSTICK_ORIENTATION == 1 ? pedestalData.gy : pedestalData.gz));

  int a;

  // pick which accelerometer axis to use
  if (JOYSTICK_ORIENTATION == 0)
  {
    a = mappedImuValues.ax;
  }
  else if (JOYSTICK_ORIENTATION == 1)
  {
    a = mappedImuValues.ay;
  }
  else
  {
    a = mappedImuValues.az;
  }

  int g;

  // pick which accelerometer axis to use
  if (JOYSTICK_ORIENTATION == 0)
  {
    g = mappedImuValues.gx;
  }
  else if (JOYSTICK_ORIENTATION == 1)
  {
    g = mappedImuValues.gy;
  }
  else
  {
    g = mappedImuValues.gz;
  }

  // check if joystick is vertical
  if (abs(a) < JOYSTICK_DEADZONE)
  {
    a = 0;    
  } 

  // Remove deadzone range from the a variable, so we immediately start moving when outside of it
  if (a > 0) 
  {
    a -= JOYSTICK_DEADZONE;
  } 
  else if (a < 0)
  {
    a += JOYSTICK_DEADZONE;
  } 

  MPUAngleSamples.add(a);
  MPUWobbleSamples.add(g);
  
  pedestalData.joystickAveragedTilt = MPUAngleSamples.getMedian();
  
  if (JOYSTICK_DIRECTION == 1) 
  {
    pedestalData.joystickAveragedTilt = 0-pedestalData.joystickAveragedTilt;
  }
  
  pedestalData.joystickPeakWobble = abs(MPUWobbleSamples.getHighest());
}

structImuValues loadMockDataInstead()
{
  structImuValues mockImuValues;

  // Vertical, untouched
  mockImuValues.ax = 6241;
  mockImuValues.ay = -15578;
  mockImuValues.az = 707;

  mockImuValues.gx = 6251;
  mockImuValues.gy = -15567;
  mockImuValues.gz = 740;

  // bent left 90, should make dot go forwards
  // mockImuValues.ax = -14657;
  // mockImuValues.ay = -3578;
  // mockImuValues.az = 5921;

  // mockImuValues.gx = -62;
  // mockImuValues.gy = 139;
  // mockImuValues.gz = -352;

  // bent right 90, should make dot go backwards
  // mockImuValues.ax = 14775;
  // mockImuValues.ay = 5880;
  // mockImuValues.az = -4572;

  // mockImuValues.gx = 14775;
  // mockImuValues.gy = 5880;
  // mockImuValues.gz = -4572;  

  return mockImuValues;
}

void sendPedestalDataOverEspNow()
{
  pedestalData.buttonPressed = !digitalRead(PIN_RESET_BUTTON);

  esp_now_send(broadcastAddress, (uint8_t *) &pedestalData, sizeof(pedestalData));
}

void debugAccelerometerValuesReadout(structImuValues mappedImuValues)
{
  #ifdef DEBUG_MODE_ON

    // Accel readouts
    Serial.print(F("aX: "));
    Serial.print(mappedImuValues.ax);
    Serial.print(F(", "));

    Serial.print(F("aY: "));
    Serial.print(mappedImuValues.ay);
    Serial.print(F(", "));

    Serial.print(F("aZ: "));
    Serial.println(mappedImuValues.az);

    // Gyro readouts
    Serial.print(F("gX: "));
    Serial.print(mappedImuValues.gx);
    Serial.print(F(", "));

    Serial.print(F("gY: "));
    Serial.print(mappedImuValues.gy);
    Serial.print(F(", "));

    Serial.print(F("gZ: "));
    Serial.println(mappedImuValues.gz);

    // Button readout
    
    Serial.print(F("Button: "));
    Serial.println(digitalRead(PIN_RESET_BUTTON));
    
    Serial.println();
    Serial.println();

  #endif
}

void initializeEspNow()
{
  
  WiFi.mode(WIFI_STA);

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
}