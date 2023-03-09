#include <FastLED.h>

// How many leds in your strip?
#define NUM_LEDS 200

// For led chips like Neopixels, which have a data line, ground, and power, you just
// need to define DATA_PIN.  For led chipsets that are SPI based (four wires - data, clock,
// ground, and power), like the LPD8806 define both DATA_PIN and CLOCK_PIN

// Define the array of leds
CRGB leds[NUM_LEDS];

void setup() {
  // Uncomment/edit one of the following lines for your leds arrangement.
  // FastLED.addLeds<TM1803, DATA_PIN, RGB>(leds, NUM_LEDS);
  // FastLED.addLeds<TM1804, DATA_PIN, RGB>(leds, NUM_LEDS);
  // FastLED.addLeds<TM1809, DATA_PIN, RGB>(leds, NUM_LEDS);
  // FastLED.addLeds<WS2811, DATA_PIN, RGB>(leds, NUM_LEDS);
  // FastLED.addLeds<WS2812, DATA_PIN, RGB>(leds, NUM_LEDS);
  // FastLED.addLeds<WS2812B, DATA_PIN, RGB>(leds, NUM_LEDS);
  // FastLED.addLeds<NEOPIXEL, DATA_PIN>(leds, NUM_LEDS);
  // FastLED.addLeds<APA104, DATA_PIN, RGB>(leds, NUM_LEDS);
  // FastLED.addLeds<UCS1903, DATA_PIN, RGB>(leds, NUM_LEDS);
  // FastLED.addLeds<UCS1903B, DATA_PIN, RGB>(leds, NUM_LEDS);
  // FastLED.addLeds<GW6205, DATA_PIN, RGB>(leds, NUM_LEDS);
  // FastLED.addLeds<GW6205_400, DATA_PIN, RGB>(leds, NUM_LEDS);

  // FastLED.addLeds<WS2801, RGB>(leds, NUM_LEDS);
  // FastLED.addLeds<SM16716, RGB>(leds, NUM_LEDS);
  // FastLED.addLeds<LPD8806, RGB>(leds, NUM_LEDS);
  // FastLED.addLeds<P9813, RGB>(leds, NUM_LEDS);
  // FastLED.addLeds<APA102, RGB>(leds, NUM_LEDS);
  // FastLED.addLeds<DOTSTAR, RGB>(leds, NUM_LEDS);

  // FastLED.addLeds<WS2801, DATA_PIN, CLOCK_PIN, RGB>(leds, NUM_LEDS);
  // FastLED.addLeds<SM16716, DATA_PIN, CLOCK_PIN, RGB>(leds, NUM_LEDS);
  // FastLED.addLeds<LPD8806, DATA_PIN, CLOCK_PIN, RGB>(leds, NUM_LEDS);
  // FastLED.addLeds<P9813, DATA_PIN, CLOCK_PIN, RGB>(leds, NUM_LEDS);
  FastLED.addLeds<APA102, DATA_PIN, CLOCK_PIN, BGR>(leds, NUM_LEDS);
  // FastLED.addLeds<DOTSTAR, DATA_PIN, CLOCK_PIN, RGB>(leds, NUM_LEDS);


  Serial.begin(115200);
  Serial.println(F("Init done!"));
}

void loop() {

  const int numLeds = 200;

    
     Serial.println(F("Wipe: Red"));


  for (int i = 0; i < numLeds; i++){
      // Turn the LED on, then pause
    leds[i] = CRGB::Red;
    FastLED.show();
    delay(5);

  }

     Serial.println(F("Wipe: Black"));

  for (int i = 0; i < numLeds; i++){
    // Now turn the LED off, then pause
    leds[i] = CRGB::Black;
    FastLED.show();
    delay(5);
    
  }

    
     Serial.println(F("Wipe: Green"));

  for (int i = 0; i < numLeds; i++){
  // Turn the LED on, then pause
  leds[i] = CRGB::Green;
  FastLED.show();
  delay(5);
  }


    
     Serial.println(F("Wipe: Black"));
  
  for (int i = 0; i < numLeds; i++){
  // Now turn the LED off, then pause
  leds[i] = CRGB::Black;
  FastLED.show();
  delay(500);
  }
  
  
    
     Serial.println(F("First led: BLUE"));

  // Turn the LED on, then pause
  leds[0] = CRGB::Blue;
  FastLED.show();
  delay(500);
  // Now turn the LED off, then pause

     Serial.println(F("First led: BLACK"));

  leds[0] = CRGB::Black;
  FastLED.show();
  delay(500);
}
