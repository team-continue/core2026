#pragma once

#include <stdint.h>

void rst_setup(void);
void rst_low(void);
void rst_high(void);

uint8_t is_esc_reset(void);