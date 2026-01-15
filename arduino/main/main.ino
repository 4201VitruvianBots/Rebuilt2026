#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
#include <avr/power.h> // Required for 16 MHz Adafruit Trinket
#endif

#define DELAY_INTERVAL 250 // 250ms pause between each pixel

#define PIN_WS2812B  2  // Arduino pin that connects to WS2812B
#define NUM_PIXELS  256  // The number of LEDs (pixels) on WS2812B

Adafruit_NeoPixel WS2812B(NUM_PIXELS, PIN_WS2812B, NEO_GRB + NEO_KHZ800);

void setup() {
  // put your setup code here, to run once:
  WS2812B.begin(); // INITIALIZE WS2812B strip object (REQUIRED)
  WS2812B.setBrightness(20); // a value from 0 to 255

}

void loop() {
  // put your main code here, to run repeatedly:
  WS2812B.clear();  // set all pixel colors to 'off'. It only takes effect if pixels.show() is called

  // turn pixels to green one by one with delay between each pixel
  for (int pixel = 0; pixel < NUM_PIXELS; pixel++) {           // for each pixel
    WS2812B.setPixelColor(pixel, WS2812B.Color(0, 255, 0));  // it only takes effect if pixels.show() is called
    WS2812B.show();                                           // send the updated pixel colors to the WS2812B hardware.

    delay(DELAY_INTERVAL);  // pause between each pixel
  }

  // turn off all pixels for two seconds
  WS2812B.clear();
  WS2812B.show();  // send the updated pixel colors to the WS2812B hardware.
  delay(2000);     // off time

  // turn on all pixels to red at the same time for two seconds
  for (int pixel = 0; pixel < NUM_PIXELS; pixel++) {           // for each pixel
    WS2812B.setPixelColor(pixel, WS2812B.Color(255, 0, 0));  // it only takes effect if pixels.show() is called
  }
  WS2812B.show();  // send the updated pixel colors to the WS2812B hardware.
  delay(2000);     // on time

  // turn off all pixels for one seconds
  WS2812B.clear();
  WS2812B.show();  // send the updated pixel colors to the WS2812B hardware.
  delay(2000);     // off time
}
