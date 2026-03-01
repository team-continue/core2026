#include "esc.h"
#include "pin.h"

void setup(){
  esc.init();
  esc.write(1500);
}

void loop(){
  esc.write(2000);
  delay(2000);
  esc.write(1000);
  delay(2000);
}