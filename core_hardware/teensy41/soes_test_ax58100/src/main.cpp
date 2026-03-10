#include "ecat_app.h"
#include <Arduino.h>

void setup (){
    pinMode(LED_BUILTIN, OUTPUT);
    Serial.begin(115200);
    ecatapp_init();
}

void loop (){
    ecatapp_loop();
}