#pragma once

#include <Arduino.h>
#include <SPI.h>

#define PIN_EMERGENCY 32
#define PORT_WIRELESS Serial5
#define LEN_WIRELESS 1024

#define SERVO_SERIAL Serial7

// #define PIN_ESC1 35
#define PIN_ESC2 24

#define SPIX_ESC SPI1

#define ESC_GPIOX_MOSI 26
#define ESC_GPIOX_MISO 39
#define ESC_GPIOX_SCK 27
#define ESC_GPIOX_CS 38

#define ESC_GPIOX_RSTN 34
#define ESC_GPIOX_IRQ 37
#define ESC_GPIOX_SYNC0 6
#define ESC_GPIOX_SYNC1 7

#define LED_UPPER_SERIAL_PIN 24
