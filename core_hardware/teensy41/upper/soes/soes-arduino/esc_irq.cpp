#include "esc_irq.h"
#include "pinout.h"
#include <Arduino.h>

void irq_interrupt_enable(void){
    // attachInterrupt(ESC_GPIOX_IRQ, irq_interrupt_callback, FALLING);
}
void irq_interrupt_disable(void){
    // detachInterrupt(ESC_GPIOX_IRQ);
}

void sync0_interrupt_enable(void){
    // attachInterrupt(ESC_GPIOX_SYNC0, sync0_interrupt_callback, FALLING);
}
void sync0_interrupt_disable(void){
    // detachInterrupt(ESC_GPIOX_SYNC0);
}
