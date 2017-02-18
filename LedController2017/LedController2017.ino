//#define RAINBOW

#include <Adafruit_NeoPixel.h>

#define FRONT_RING_PIN  8
#define REAR_RING_PIN  7
#define NUMPIXELS_RING  16

#define LIGHT_PIN 6
#define NUMPIXELS_STRIPS 64

#define RINGFADE 35

Adafruit_NeoPixel rearpixels = Adafruit_NeoPixel(NUMPIXELS_RING, REAR_RING_PIN, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel frontpixels = Adafruit_NeoPixel(NUMPIXELS_RING, FRONT_RING_PIN, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel strippixels = Adafruit_NeoPixel(NUMPIXELS_STRIPS, LIGHT_PIN, NEO_GRB + NEO_KHZ800);

void setup() {
  rearpixels.begin();
  frontpixels.begin();
  strippixels.begin();
  Serial.begin(115200);

  pinMode(13,OUTPUT);
}

byte rainbow = 0;
byte spin = 0;
long lastread = 0;
byte add = 0;

int pulse = 0;
boolean pulsedir = false;

byte front_leds = false, rear_leds = false, enabled = false, auton = false, teleop = false, red = false, blue = false;

void loop() {
  if (Serial.available()) {
    byte c = Serial.read();
    lastread = millis();
    front_leds = c & 0b00001000;
    rear_leds = c & 0b00000100;
    auton = c & 0b00100000;
    teleop = c & 0b00010000;
    red = c & 0b10000000;
    blue = c & 0b01000000;
    enabled = auton || teleop;
    digitalWrite(13,HIGH);
  }
  if (lastread < millis() - 1000) { // lost comms
    front_leds = false;
    rear_leds = false;
    enabled = false;
    auton = false;
    teleop = false;
    red = false;
    blue = false;
    digitalWrite(13,LOW);
  }

  front_leds = true;
  rear_leds = true;
  
  rainbow+=4; // this will automatically wrap around
  spin++;
  spin %= NUMPIXELS_RING;
  if (pulsedir) {
    pulse--;
  } else {
    pulse++;
  }
  if (pulse > NUMPIXELS_STRIPS) pulsedir = true;
  if (pulse < 0) pulsedir = false;

  if (auton) {
    if (red) {
      for (byte i = 0; i < NUMPIXELS_STRIPS; i++)
        strippixels.setPixelColor(i,strippixels.Color(pulse,0,0));
    } else if (blue) {
      for (byte i = 0; i < NUMPIXELS_STRIPS; i++)
        strippixels.setPixelColor(i,strippixels.Color(0,0,pulse));
    }
  } else if (teleop) {
    fade_s(10);
    if (red) {
      strippixels.setPixelColor(pulse,strippixels.Color(255,0,0));
    } else if (blue) {
      strippixels.setPixelColor(pulse,strippixels.Color(0,0,255));
    }
  } else {
    #ifdef RAINBOW
    for (byte i = 0; i < NUMPIXELS_STRIPS; i++) {
      uint32_t color = wheel(((i * 255 / NUMPIXELS_STRIPS) + rainbow) & 255);
      strippixels.setPixelColor(i,color);
    }
    #else
    fade_s(3);
    add++;
      if (add > 2) {
        add = 0;
        strippixels.setPixelColor(random(NUMPIXELS_STRIPS),wheel(random(0xff)));
      }
    #endif
  }

  if (front_leds) {
    for (byte i = 0; i < NUMPIXELS_RING; i++) {
      frontpixels.setPixelColor(i,frontpixels.Color(0,255,0));
    }
  } else {
    #ifdef RAINBOW
    for (byte i = 0; i < NUMPIXELS_RING; i++) {
      uint32_t color = wheel(((i * 255 / NUMPIXELS_RING) + rainbow) & 255);
      frontpixels.setPixelColor(i,color);
    }
    #else
    fade_f(RINGFADE);
    frontpixels.setPixelColor(spin,frontpixels.Color(255,0,0));
    #endif
  }
  
  if (rear_leds) {
    for (byte i = 0; i < NUMPIXELS_RING; i++) {
      rearpixels.setPixelColor(i,rearpixels.Color(0,255,0));
    }
  } else {
    #ifdef RAINBOW
    for (byte i = 0; i < NUMPIXELS_RING; i++) {
      uint32_t color = wheel(((i * 255 / NUMPIXELS_RING) + rainbow) & 255);
      rearpixels.setPixelColor(i,color);
    }
    #else
    fade_r(RINGFADE);
    rearpixels.setPixelColor(spin,rearpixels.Color(255,0,0));
    #endif
  }

  frontpixels.show();
  rearpixels.show();
  strippixels.show();

  delay(40);
}

uint32_t wheel(byte WheelPos) {
  WheelPos = 255 - WheelPos;
  if(WheelPos < 85) {
   return frontpixels.Color(255 - WheelPos * 3, 0, WheelPos * 3);
  } else if(WheelPos < 170) {
    WheelPos -= 85;
   return frontpixels.Color(0, WheelPos * 3, 255 - WheelPos * 3);
  } else {
   WheelPos -= 170;
   return frontpixels.Color(WheelPos * 3, 255 - WheelPos * 3, 0);
  }
}

void fade_f(byte FADE){
  for (byte i = 0; i < NUMPIXELS_RING; i++) {
    uint32_t pcolor = frontpixels.getPixelColor(i);
    uint8_t red,green,blue;
    blue = pcolor;
    green = pcolor>>8;
    red = pcolor>>16;
    if (red>FADE) {
      red-=FADE;
    } else {
      red = 0;
    }
    if (green>FADE) {
      green-=FADE;
    } else {
      green = 0;
    }
    if (blue>FADE) {
      blue-=FADE;
    } else {
      blue = 0;
    }
    frontpixels.setPixelColor(i,red,green,blue);
  }
}

void fade_r(byte FADE){
  for (byte i = 0; i < NUMPIXELS_RING; i++) {
    uint32_t pcolor = rearpixels.getPixelColor(i);
    uint8_t red,green,blue;
    blue = pcolor;
    green = pcolor>>8;
    red = pcolor>>16;
    if (red>FADE) {
      red-=FADE;
    } else {
      red = 0;
    }
    if (green>FADE) {
      green-=FADE;
    } else {
      green = 0;
    }
    if (blue>FADE) {
      blue-=FADE;
    } else {
      blue = 0;
    }
    rearpixels.setPixelColor(i,red,green,blue);
  }
}

void fade_s(byte FADE){
  for (byte i = 0; i < NUMPIXELS_STRIPS; i++) {
    uint32_t pcolor = strippixels.getPixelColor(i);
    uint8_t red,green,blue;
    blue = pcolor;
    green = pcolor>>8;
    red = pcolor>>16;
    if (red>FADE) {
      red-=FADE;
    } else {
      red = 0;
    }
    if (green>FADE) {
      green-=FADE;
    } else {
      green = 0;
    }
    if (blue>FADE) {
      blue-=FADE;
    } else {
      blue = 0;
    }
    strippixels.setPixelColor(i,red,green,blue);
  }
}

