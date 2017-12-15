#ifndef LIGHTS_H
#define LIGHTS_H

#include <Arduino.h>
//#include "dotstar.h"
#include "FastLED.h"
#include <SPI.h>
#include "system_info.h"

class Lights
{ //In Which We Light The World
public:
  //Lights();
  ~Lights(){};
  uint8_t maxBrightness = 128; //out of 255
  bool _on = true;


  //Counting variables
  uint8_t whl;

  int active_strip = 0;
  CRGB _led; // For Looping
  CRGB active_leds[NUM_LEDS];
  CRGB leds[NUM_STRIPS][NUM_LEDS];

  //SPI Update Methods
  void writeStrip(CRGB ledBuffer []);
  uint8_t brightness = 0b11111;
  uint8_t startFrame[4]  ={0x00,0x00,0x00,0x00};
  uint8_t endFrame[4]    ={0xFF,0xFF,0xFF,0xFF};
  uint8_t brightnessFrame= 0b111 | brightness;

  void strip0_active();
  void strip1_active();
  void strip2_active();
  void toggleStrip();
  void activateStrip();

  //Important Funcitons
  virtual void update(uint8_t wait);
  virtual void initlaize();
  void refresh();


  void fillActiveStrip(CRGB color);
  void fillActiveStrip(CRGB color, CRGB tipColor);

  //Color Functions
  void colorAll(CRGB color);
  void allOff();
  void red();
  void green();
  void blue();
};


#endif //LIGHTS_H
