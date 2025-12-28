#include "ecat_app.h"
#include <Arduino.h>

void setup (){
    Serial.begin(9600);
    ecatapp_init();
}

void loop (){
    ecatapp_loop();
}