#include <stdint.h>
#include <spi_hw.h>

#include <Arduino.h>
#include <SPI.h>

#include "pinout.h"

void spi_setup(void){
    SPIX_ESC.begin();
    SPIX_ESC.setMOSI(ESC_GPIOX_MOSI);
    SPIX_ESC.setMISO(ESC_GPIOX_MISO);
    SPIX_ESC.setSCK(ESC_GPIOX_SCK);
    pinMode(ESC_GPIOX_CS, OUTPUT);
}
void spi_select (int8_t board){
    digitalWrite(ESC_GPIOX_CS, LOW);
}
void spi_unselect (int8_t board){
    digitalWrite(ESC_GPIOX_CS, HIGH);
}
void spi_write (int8_t board, uint8_t *data, uint8_t size){
    SPIX_ESC.transfer(data, NULL, size);
}
void spi_read (int8_t board, uint8_t *result, uint8_t size){
    SPIX_ESC.transfer(NULL, result, size);
}
void spi_bidirectionally_transfer (int8_t board, uint8_t *result, uint8_t *data, uint8_t size){
    SPIX_ESC.transfer(data, result, size);
}