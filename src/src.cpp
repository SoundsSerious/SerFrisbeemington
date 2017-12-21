#include <WiFi.h>
#include <Arduino.h>
#include <SPI.h>
#include <SimpleBLE.h>
#include <FastLED.h>
#include <ESPAsyncWebServer.h>
#include "globals.h"


//using namespace std;
//WARNING: I SUCK AT SPELLING. FEEL FREE TO FIX ANY SPELLING ERRORS XP

Frisbeem frisbeem;

void on_off_buttonaction() {
	frisbeem.com.log("BUTTON 0 PRESSED",true);
	frisbeem.lights._on = !frisbeem.lights._on;
}

void setup() {

delay(100);
frisbeem.initlaize();
//frisbeem._com.log("Finish Setup");

pinMode(BUTTON, INPUT_PULLUP);
attachInterrupt(digitalPinToInterrupt(BUTTON),on_off_buttonaction, RISING);

}

void loop() {

frisbeem.update();
}
