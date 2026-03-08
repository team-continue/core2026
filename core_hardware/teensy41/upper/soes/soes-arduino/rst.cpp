#include "rst.h"
#include "pinout.h"
#include <Arduino.h>

void rst_setup(void){
    pinMode(ESC_GPIOX_RSTN, OUTPUT);
    rst_high();
}

void rst_low(void){
    digitalWrite(ESC_GPIOX_RSTN, LOW);
}

void rst_high(void){
    digitalWrite(ESC_GPIOX_RSTN, HIGH);
}

uint8_t is_esc_reset(void){
    /* Check if ESC pulled RSTN line up */ 
    return (digitalRead(ESC_GPIOX_RSTN) == HIGH);
}

