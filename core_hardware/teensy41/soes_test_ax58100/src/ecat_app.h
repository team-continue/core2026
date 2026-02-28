#pragma once

void ecatapp_init();
void ecatapp_loop();

void ecatapp_setUint8(const uint8_t id, const uint8_t *data, const uint8_t len);
void ecatapp_setFloat(const uint8_t id, const float *data, const uint8_t len);
void ecatapp_recv_cb(const uint8_t id, const uint8_t* data, const uint8_t uint8_len);