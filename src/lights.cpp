#include "lights.h"
#include "globals.h"

SPIClass SPI2 = SPIClass(HSPI);
SPISettings APA102SPI(LED_SPI_CLOCK, MSBFIRST, SPI_MODE3);

void Lights::initlaize()
{
  pinMode(S_C0_MUXPIN, OUTPUT);
  pinMode(S_C1_MUXPIN, OUTPUT);
  pinMode(S_D0_MUXPIN, OUTPUT);
  pinMode(S_D1_MUXPIN, OUTPUT);
  SPI2.begin(LED_CLOCK_PIN, LED_MISO_PIN, LED_DATA_PIN, LED_CS_PIN);
}

void Lights::update(uint8_t wait)
{
  if (_on){
    if ( frisbeem.mpu.rest ){ colorAll(CRGB(0,0,255)); }
    else if(frisbeem.mpu.spin ){ colorAll(CRGB(255,0,0)); }
    else if( ~frisbeem.mpu.rest ){ colorAll(CRGB(0,0,255)); }
    refresh();
  }
  else{
    allOff();
  }

}

void Lights::refresh(){


}

// Set all pixels in the strip to a solid color, then wait (ms)
void Lights::red() {
  uint16_t i;
  for(i=0; i<NUM_STRIPS; i++) {
    toggleStrip();
    fill_solid( &active_leds[0], NUM_LEDS, CRGB(100,0,0) );
    writeStrip( &active_leds[0]);
  }
}

void Lights::blue() {
  uint16_t i;
  for(i=0; i<NUM_STRIPS; i++) {
    toggleStrip();
    fill_solid( &active_leds[0], NUM_LEDS, CRGB(0,0,50) );
    writeStrip( &active_leds[0]);
  }
}

void Lights::green() {
  uint16_t i;
  for(i=0; i<NUM_STRIPS; i++) {
    toggleStrip();
    fill_solid( &active_leds[0], NUM_LEDS, CRGB(0,50,0) );
    writeStrip( &active_leds[0]);
  }
}


// Set all pixels in the strip to a solid color, then wait (ms)
void Lights::colorAll(CRGB color) {
  uint16_t i;
  for(i=0; i<NUM_STRIPS; i++) {
    toggleStrip();
    fill_solid( &active_leds[0], NUM_LEDS, color );
    writeStrip( &active_leds[0]);
  }
}

// Set all pixels in the strip to a solid color, then wait (ms)
void Lights::allOff() {
  uint16_t i;
  for(i=0; i<NUM_STRIPS; i++) {
    toggleStrip();
    fill_solid( &active_leds[0], NUM_LEDS, CRGB(0,0,0) );
    writeStrip( &active_leds[0]);

  }
}

//Strip Utility Functions
void Lights::strip0_active(){
  digitalWrite(S_C0_MUXPIN,LOW);
  digitalWrite(S_C1_MUXPIN,LOW);
	digitalWrite(S_D0_MUXPIN,LOW);
  digitalWrite(S_D1_MUXPIN,LOW);
  active_strip = 0;
}

void Lights::strip1_active(){
	digitalWrite(S_C0_MUXPIN,HIGH);
  digitalWrite(S_C1_MUXPIN,LOW);
	digitalWrite(S_D0_MUXPIN,HIGH);
  digitalWrite(S_D1_MUXPIN,LOW);
  active_strip = 1;
}

void Lights::strip2_active(){
	digitalWrite(S_C0_MUXPIN,LOW);
  digitalWrite(S_C1_MUXPIN,HIGH);
	digitalWrite(S_D0_MUXPIN,LOW);
  digitalWrite(S_D1_MUXPIN,HIGH);
  active_strip = 2;
}


void Lights::toggleStrip(){
  active_strip += 1;
  if (active_strip > 2){
    active_strip = 0;
  }
  if (active_strip == 0){
    strip0_active();
  }
  else if (active_strip == 1){
    strip1_active();
  }
  else if (active_strip == 2){
    strip2_active();
  }
  activateStrip();
}

void Lights::fillActiveStrip( CRGB color){
  fill_solid( &active_leds[0], NUM_LEDS, color );
  writeStrip( &active_leds[0]);
}
void Lights::fillActiveStrip(CRGB color, CRGB tipColor){
  fill_solid( &active_leds[0], NUM_LEDS, color );
  active_leds[NUM_LEDS-1] = tipColor;
  writeStrip( &active_leds[0]);
}

void Lights::writeStrip(CRGB ledBuffer []){
	SPI2.beginTransaction(APA102SPI);
  //Use After SPI Begin
  SPI2.transfer(0x00);
  SPI2.transfer(0x00);
  SPI2.transfer(0x00);
  SPI2.transfer(0x00);
  for (int i=0; i <= NUM_LEDS; i++){
    _led = ledBuffer[i];
    SPI2.transfer(brightnessFrame);
    SPI2.transfer(_led.b);
    SPI2.transfer(_led.g);
    SPI2.transfer(_led.r);
  }
  SPI2.transfer(0xFF);
  SPI2.transfer(0xFF);
  SPI2.transfer(0xFF);
  SPI2.transfer(0xFF);
	SPI2.endTransaction();
}

void Lights::activateStrip(){
  //First Set Active LED
  memcpy(active_leds, leds[active_strip], sizeof(active_leds));
}
