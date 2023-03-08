#include "FastLED.h"

// How many leds in your strip?
#define NUM_LEDS 692
#define DATA_PIN 13
#define CLOCK_PIN 11

// Define the array of leds
CRGB leds[NUM_LEDS];

void setup() { 
	//Serial.begin(115200);
	Serial.println("resetting");
	FastLED.addLeds<APA102,13,11,BGR,DATA_RATE_MHZ(3)>(leds,NUM_LEDS);
	LEDS.setBrightness(84);
}

int sweep = 0;

void loop() { 
  for(int dot = 0; dot < NUM_LEDS; dot++) { 
      leds[dot] = CRGB::Red;      
      FastLED.show();
      // clear this led for the next time around the loop
      leds[dot] = CRGB::Black;
      delay(30);
  }    
}
