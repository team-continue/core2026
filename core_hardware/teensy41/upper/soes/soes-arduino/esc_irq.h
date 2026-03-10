#pragma once

#include <stdint.h>

void ESC_interrupt_enable (uint32_t mask);
void ESC_interrupt_disable (uint32_t mask);

void irq_interrupt_enable(void);
void irq_interrupt_disable(void);
// void irq_interrupt_callback(void);

void sync0_interrupt_enable(void);
void sync0_interrupt_disable(void);
// void sync0_interrupt_callback(void);

uint32_t ESC_enable_DC (void);