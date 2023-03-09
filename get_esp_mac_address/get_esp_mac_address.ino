#if defined(ESP8266)
  #include <ESP8266WiFi.h>
#elif defined(ESP32)
  #include <WiFi.h>
#else
  #error "This ain't a ESP8266 or ESP32, foolish mortal!"
#endif

void setup()
{
  Serial.begin(115200);
}
 
void loop() 
{
  Serial.println();
  Serial.println();
  Serial.println();
  Serial.println();
  Serial.println();
  Serial.println();

  #if defined(ESP8266)
    Serial.print("ESP8266 MAC Address:  ");
  #elif defined(ESP32)
    Serial.print("ESP32 MAC Address:  ");
  #endif

  Serial.println(WiFi.macAddress()); 

  delay(2000);
}