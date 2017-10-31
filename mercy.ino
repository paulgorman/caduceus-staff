// Overwatch: Mercy's caduceus staff

#include <Adafruit_NeoPixel.h>

#define PIN  0
#define LED  1
#define NUM_LEDS  19
#define BUTTON 2

Adafruit_NeoPixel pixels = Adafruit_NeoPixel(NUM_LEDS, PIN);
     
uint8_t  mode   = 0,        // Current animation effect
offset = 0;        // Position of spinner animation
uint32_t colorYellow  = 0xFF8000; // amber
uint32_t colorBlue = 0x1100FF; // blue
uint32_t prevTime;          // Time of last animation mode switch
long debounce = 400; // the debounce time
long Btime = 0;
     
void setup() {
	pinMode(LED, OUTPUT);
	pinMode(BUTTON, INPUT_PULLUP);
	pixels.begin();
	pixels.setBrightness(100); 
	prevTime = millis();      // Starting time
}

void loop() {
	uint8_t  i;
 	uint8_t  j;
 	uint8_t  k;
 	uint32_t t;

	if (digitalRead(BUTTON) == LOW && millis() - Btime > debounce) {
 		if (mode == 2) {
			mode = 0;
		} else {
			mode = mode + 1;
		}
		for (i=0; i<NUM_LEDS; i++) {
			uint32_t c = 0;
			pixels.setPixelColor(i,c);
		}
		pixels.show();
		Btime = millis();    
	}
     
	switch(mode) {

		case 1: // Random sparkles - just one LED on at a time!
        i = random(NUM_LEDS);           // Choose a random pixel
        pixels.setPixelColor(i, colorBlue); // Set it to current color
        pixels.setPixelColor(i+1, colorBlue); // Set it to current color
        pixels.setPixelColor(i-1, colorBlue); // Set it to current color
        j = random(NUM_LEDS);           // Choose a random pixel
        pixels.setPixelColor(j, colorBlue); // Set it to current color
        pixels.setPixelColor(j+1, colorBlue); // Set it to current color
        pixels.setPixelColor(j-1, colorBlue); // Set it to current color
        k = random(NUM_LEDS);           // Choose a random pixel
        pixels.setPixelColor(k, colorBlue); // Set it to current color
        pixels.setPixelColor(k+1, colorBlue); // Set it to current color
        pixels.setPixelColor(k-1, colorBlue); // Set it to current color
        pixels.setPixelColor(0,colorBlue);  // always want center handle status on
        pixels.show();                  // Refresh LED states
        // Set same pixel to "off" color now but DON'T refresh...
        // it stays on for now...both this and the next random
        // pixel will be refreshed on the next pass.
        pixels.setPixelColor(i, 0);
        pixels.setPixelColor(i+1, 0);
        pixels.setPixelColor(i-1, 0);
        pixels.setPixelColor(j, 0);
        pixels.setPixelColor(j+1, 0);
        pixels.setPixelColor(j-1, 0);
        pixels.setPixelColor(k, 0);
        pixels.setPixelColor(k+1, 0);
        pixels.setPixelColor(k-1, 0);
        delay(100);                      // 10 millisecond delay
        break;
     
		case 0: // Spinny wheel (4 LEDs on at a time)
        for(i=0; i<NUM_LEDS; i++) {    // For each LED...
          uint32_t c = 0;              // Assume pixel will be "off" color
          if(((offset + i) & 5) < 2) { // For each 6 pixels, 2 will be...
            c = colorYellow;                 // ...assigned the current color
          }
          pixels.setPixelColor(i, c);  // Set color of pixel 'i'
        }
        pixels.setPixelColor(0,colorYellow);  // always want center handle status on
        pixels.show();                 // Refresh LED states
        delay(50);                     // 50 millisecond delay
        offset++;                      // Shift animation by 1 pixel on next frame
        break;
     
			case 2: // off
        for (i=0; i<NUM_LEDS; i++) {
          uint32_t c = 0;
          pixels.setPixelColor(i, c);
        }
        pixels.show();
        digitalWrite(LED, HIGH);    // light up the LED
        delay(200);
        digitalWrite(LED, LOW);
        break;
	}
}
