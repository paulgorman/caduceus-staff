/*********************************************************************
  PRESENCE USE ME THIS AINT BAD
  20241027
 *********************************************************************/

#include <string.h>
#include <Arduino.h>
#include <SPI.h>
#if not defined (_VARIANT_ARDUINO_DUE_X_) && not defined (_VARIANT_ARDUINO_ZERO_)
  #include <SoftwareSerial.h>
#endif

#include "Adafruit_BLE.h"
#include "Adafruit_BluefruitLE_SPI.h"
#include "Adafruit_BluefruitLE_UART.h"

#include "BluefruitConfig.h"

#include <Adafruit_NeoPixel.h>

/*=========================================================================
    -----------------------------------------------------------------------*/
    #define FACTORYRESET_ENABLE     1
    #define BODYPIN                 5
    #define PIN                     6
    #define NUMPIXELS               7
    #define BODYPIXELS              7
/*=========================================================================*/

Adafruit_NeoPixel pixel = Adafruit_NeoPixel(NUMPIXELS, PIN, NEO_GRBW + NEO_KHZ800);  // PRESENCE FINALLY YASDFASDF
Adafruit_NeoPixel bodypixel = Adafruit_NeoPixel(BODYPIXELS, BODYPIN, NEO_GRB + NEO_KHZ800);

Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_CS, BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);

// A small helper
void error(const __FlashStringHelper*err) {
  Serial.println(err);
  while (1);
}

// function prototypes over in packetparser.cpp
uint8_t readPacket(Adafruit_BLE *ble, uint16_t timeout);
float parsefloat(uint8_t *buffer);
void printHex(const uint8_t * data, const uint32_t numBytes);

// the packet buffer
extern uint8_t packetbuffer[];

/**************************************************************************/
/*!
    @brief  Sets up the HW an the BLE module (this function is called
            automatically on startup)
*/
/**************************************************************************/

uint8_t red = 255;
uint8_t green = 128;
uint8_t blue = 0;
uint8_t animationState = 1;
int pos = 0, dir = 1; // Position, direction of "eye" for larson scanner animation

void setup(void)
{
  //while (!Serial);  // required for Flora & Micro
  delay(500);

  // turn off neopixel
  pixel.begin(); // This initializes the NeoPixel library.
  bodypixel.begin(); // This initializes the NeoPixel library.

  for(uint8_t i=0; i<NUMPIXELS; i++) {
    pixel.setPixelColor(i, pixel.Color(0,0,0)); // off
  }
  for(uint8_t i=0; i<BODYPIXELS; i++) {
    bodypixel.setPixelColor(i, bodypixel.Color(0,0,0)); // off
  }
  colorWipe(pixel.Color(100, 100, 100),20); // do a quick colorWipe to show that the pixels are all working, even before Bluefruit connection established
  colorWipe(pixel.Color(0, 0, 0),20); 

  Serial.begin(9600);
  Serial.println(F("Adafruit Bluefruit Neopixel Color Picker Example"));
  Serial.println(F("------------------------------------------------"));

  /* Initialise the module */
  Serial.print(F("Initialising the Bluefruit LE module: "));

  if ( !ble.begin(VERBOSE_MODE) )
  {
    error(F("Couldn't find Bluefruit, make sure it's in CoMmanD mode & check wiring?"));
  }
  Serial.println( F("OK!") );

  if ( FACTORYRESET_ENABLE )
  {
    /* Perform a factory reset to make sure everything is in a known state */
    Serial.println(F("Performing a factory reset: "));
    if ( ! ble.factoryReset() ){
      error(F("Couldn't factory reset"));
    }
  }

  /* Disable command echo from Bluefruit */
  ble.echo(false);

  Serial.println("Requesting Bluefruit info:");
  /* Print Bluefruit information */
  ble.info();

  Serial.println(F("Please use Adafruit Bluefruit LE app to connect in Controller mode"));
  Serial.println(F("Then activate/use the sensors, color picker, game controller, etc!"));
  Serial.println();

  ble.verbose(false);  // debug info is a little annoying after this point!

  /* Wait for connection */
  while (! ble.isConnected()) {
      delay(500);
  }

  Serial.println(F("***********************"));

  // Set Bluefruit to DATA mode
  Serial.println( F("Switching to DATA mode!") );
  ble.setMode(BLUEFRUIT_MODE_DATA);

  Serial.println(F("***********************"));

}

/**************************************************************************/
/*!
    @brief  Constantly poll for new command or response data
*/
/**************************************************************************/
void loop(void)
{
  /* Wait for new data to arrive */
  uint8_t len = readPacket(&ble, BLE_READPACKET_TIMEOUT);
  //if (len == 0) return;

  // Color
  if (packetbuffer[1] == 'C') {
    red = packetbuffer[2];
    green = packetbuffer[3];
    blue = packetbuffer[4];
    Serial.print ("RGB #");
    if (red < 0x10) Serial.print("0");
    Serial.print(red, HEX);
    if (green < 0x10) Serial.print("0");
    Serial.print(green, HEX);
    if (blue < 0x10) Serial.print("0");
    Serial.println(blue, HEX);
  }

  // Buttons
  if (packetbuffer[1] == 'B') {
 
    uint8_t buttnum = packetbuffer[2] - '0';
    boolean pressed = packetbuffer[3] - '0';
    Serial.print ("Button "); Serial.print(buttnum);
    animationState = buttnum;
    if (pressed) {
      Serial.println(" pressed");
    } else {
      Serial.println(" released");
    }
  }
    
  if (animationState == 1) { // button labeled "1" in control pad
     colorWipe(pixel.Color(red, green, blue),60);
     colorWipe(pixel.Color(0, 0, 0),60);
  }
  
  if (animationState == 2) { // button labeled "2" in control pad
    for(uint16_t i=0; i<pixel.numPixels(); i++) { //clear all pixels before displaying new animation
          pixel.setPixelColor(i, pixel.Color(0,0,0));
    }
    for(uint16_t i=0; i<bodypixel.numPixels(); i++) { //clear all pixels before displaying new animation
      pixel.setPixelColor(i, bodypixel.Color(0,0,0));
    }
    colorWipe(pixel.Color(red, green, blue),40);
    colorWipe(pixel.Color(0, 0, 0),20);
  }

  if (animationState == 3) { // button labeled "3" in control pad
    larsonScanner(60); // larsonScanner is set to red and does not take color input.
  }
  
  if (animationState == 4) { // button labeled "4" in control pad
    for(uint16_t i=0; i<pixel.numPixels(); i++) { //clear all pixels before displaying new animation
          pixel.setPixelColor(i, pixel.Color(0,0,0));
        }
    rainbowCycle(10);
  }
}

void colorWipe(uint32_t c, uint8_t wait) {
  for(uint16_t i=0; i<pixel.numPixels(); i++) {
      pixel.setPixelColor(i, c);
      bodypixel.setPixelColor(i, c);
      pixel.show();
      bodypixel.show();
      delay(wait);
  }
}

void larsonScanner(uint8_t wait){
  int j;
  for(uint16_t i=0; i<pixel.numPixels()+3; i++) {
    // Draw 3 pixels centered on pos.  setPixelColor() will clip any
    // pixels off the ends of the strip, we don't need to watch for that.
    pixel.setPixelColor(pos - 2, 0x5c2e00); // Dark red
    pixel.setPixelColor(pos - 1, 0xa35200); // Medium red
    pixel.setPixelColor(pos    , 0xFF8800); // Center pixel is brightest
    pixel.setPixelColor(pos + 1, 0xa35200); // Medium red
    pixel.setPixelColor(pos + 2, 0x5c2e00); // Dark red
    bodypixel.setPixelColor(pos - 2, 0x5c2e00); // Dark red
    bodypixel.setPixelColor(pos - 1, 0xa35200); // Medium red
    bodypixel.setPixelColor(pos    , 0xFF8800); // Center pixel is brightest
    bodypixel.setPixelColor(pos + 1, 0xa35200); // Medium red
    bodypixel.setPixelColor(pos + 2, 0x5c2e00); // Dark red
    pixel.show();
    bodypixel.show();
    delay(wait);
    // Rather than being sneaky and erasing just the tail pixel,
    // it's easier to erase it all and draw a new one next time.
    for(j=-2; j<= 2; j++) pixel.setPixelColor(pos+j, 0);
    for(j=-2; j<= 2; j++) bodypixel.setPixelColor(pos+j, 0);
    // Bounce off ends of strip
    pos += dir;
    if(pos < 0) {
      pos = 1;
      dir = -dir;
    } else if(pos >= pixel.numPixels()) {
      pos = pixel.numPixels() - 2;
      dir = -dir;
    } 
  } 
}

void flashRandom(int wait, uint8_t howmany) {
 randomSeed(analogRead(0));
  for(uint16_t i=0; i<howmany; i++) {
    // get a random pixel from the list
    int j = random(pixel.numPixels());
    
    // now we will 'fade' it in 5 steps
    for (int x=0; x < 5; x++) {
      int r = red * (x+1); r /= 5;
      int g = green * (x+1); g /= 5;
      int b = blue * (x+1); b /= 5;
      
      pixel.setPixelColor(j, pixel.Color(r, g, b));
      pixel.show();
      delay(wait);
    }
    // & fade out in 5 steps
    for (int x=5; x >= 0; x--) {
      int r = red * x; r /= 5;
      int g = green * x; g /= 5;
      int b = blue * x; b /= 5;
      
      pixel.setPixelColor(j, pixel.Color(r, g, b));
      pixel.show();
      delay(wait);
    }
  }
  // LEDs will be off when done (they are faded to 0)
}

void rainbow(uint8_t wait) {
  uint16_t i, j;

  for(j=0; j<256; j++) {
    for(i=0; i<pixel.numPixels(); i++) {
      pixel.setPixelColor(i, Wheel((i+j) & 255));
      bodypixel.setPixelColor(i, Wheel((i+j) & 255));
    }
    pixel.show();
    bodypixel.show();
    delay(wait);
  }
}

// Slightly different, this makes the rainbow equally distributed throughout
void rainbowCycle(uint8_t wait) {
  uint16_t i, j;

  for(j=0; j<256*5; j++) { // 5 cycles of all colors on wheel
    for(i=0; i< pixel.numPixels(); i++) {
      pixel.setPixelColor(i, Wheel(((i * 256 / pixel.numPixels()) + j) & 255));
      bodypixel.setPixelColor(i, Wheel(((i * 256 / pixel.numPixels()) + j) & 255));
    }
    pixel.show();
    bodypixel.show();
    delay(wait);
  }
}

//Theatre-style crawling lights.
void theaterChase(uint32_t c, uint8_t wait) {
  for (int j=0; j<10; j++) {  //do 10 cycles of chasing
    for (int q=0; q < 3; q++) {
      for (int i=0; i < pixel.numPixels(); i=i+3) {
        pixel.setPixelColor(i+q, c);    //turn every third pixel on
      }
      pixel.show();

      delay(wait);

      for (int i=0; i < pixel.numPixels(); i=i+3) {
        pixel.setPixelColor(i+q, 0);        //turn every third pixel off
      }
    }
  }
}

//Theatre-style crawling lights with rainbow effect
void theaterChaseRainbow(uint8_t wait) {
  for (int j=0; j < 256; j++) {     // cycle all 256 colors in the wheel
    for (int q=0; q < 3; q++) {
      for (int i=0; i < pixel.numPixels(); i=i+3) {
        pixel.setPixelColor(i+q, Wheel( (i+j) % 255));    //turn every third pixel on
      }
      pixel.show();

      delay(wait);

      for (int i=0; i < pixel.numPixels(); i=i+3) {
        pixel.setPixelColor(i+q, 0);        //turn every third pixel off
      }
    }
  }
}

// Input a value 0 to 255 to get a color value.
// The colours are a transition r - g - b - back to r.
uint32_t Wheel(byte WheelPos) {
  WheelPos = 255 - WheelPos;
  if(WheelPos < 85) {
    return pixel.Color(255 - WheelPos * 3, 0, WheelPos * 3);
    return bodypixel.Color(255 - WheelPos * 3, 0, WheelPos * 3);

  }
  if(WheelPos < 170) {
    WheelPos -= 85;
    return pixel.Color(0, WheelPos * 3, 255 - WheelPos * 3);
    return bodypixel.Color(0, WheelPos * 3, 255 - WheelPos * 3);
  }
  WheelPos -= 170;
  return pixel.Color(WheelPos * 3, 255 - WheelPos * 3, 0);
  return bodypixel.Color(WheelPos * 3, 255 - WheelPos * 3, 0);
}
