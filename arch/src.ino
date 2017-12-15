#include <Arduino.h>
#include "globals.h"

using namespace std;

//WARNING: I SUCK AT SPELLING. FEEL FREE TO FIX ANY SPELLING ERRORS XP

Frisbeem frisbeem;



void setup() {
delay(100);
frisbeem.initlaize();
//frisbeem._com.log("Finish Setup");

}

void loop() {

frisbeem.update();

}
