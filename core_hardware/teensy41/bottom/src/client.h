#pragma once

#include <Arduino.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define PORT_SERIAL Serial4

#define MAX_LEN 2048

byte serial4RXbuffer[MAX_LEN];

class Client{
  public:
    volatile uint8_t hp = 100;
    volatile int destoy = 0;
    volatile uint16_t node_id = 0;
    volatile uint8_t flags = 0;
    volatile uint8_t color = 0;
    volatile uint8_t voltage = 0;
    volatile uint8_t pid = 0;
    volatile uint8_t hit_flags = 0;

    char recv_line[MAX_LEN];
    size_t recv_len = 0;
    bool debug_enabled = false;
    unsigned long debug_interval_ms = 1000;
    unsigned long last_debug_print_ms = 0;

    explicit Client(bool debug_enabled_in = false, unsigned long debug_interval_ms_in = 1000)
    : debug_enabled(debug_enabled_in), debug_interval_ms(debug_interval_ms_in) {}

    void init(){
      PORT_SERIAL.begin(115200);
      PORT_SERIAL.addMemoryForRead(serial4RXbuffer, MAX_LEN);
    }

    void update(){
      while(PORT_SERIAL.available()){
        const char now = static_cast<char>(PORT_SERIAL.read());
        // Serial.print(now);

        if(now == '\r'){
          continue;
        }

        if(now == '\n'){
          recv_line[recv_len] = '\0';
          parseLine();
          recv_len = 0;
          continue;
        }

        if(recv_len < (MAX_LEN - 1)){
          recv_line[recv_len++] = now;
        }else{
          recv_len = 0;
        }
      }

      if(debug_enabled){
        const unsigned long now = millis();
        if(now - last_debug_print_ms >= debug_interval_ms){
          last_debug_print_ms = now;
          print();
        }
      }
    }

    void print() const {
      Serial.print("node_id=");
      Serial.print(node_id);
      Serial.print(" flags=0x");
      Serial.print(flags, HEX);
      Serial.print(" destroy=");
      Serial.print(destoy);
      Serial.print(" color=0x");
      Serial.print(color, HEX);
      Serial.print(" hp=");
      Serial.print(hp);
      Serial.print(" voltage=0x");
      Serial.print(voltage, HEX);
      Serial.print(" pid=0x");
      Serial.print(pid, HEX);
      Serial.print(" hit_flags=0x");
      Serial.println(hit_flags, HEX);
    }

  private:
    static bool hasExpectedCommaCount(const char *text, const size_t length, const size_t expected_count) {
      size_t comma_count = 0;
      for(size_t i = 0; i < length; ++i){
        if(text[i] == ','){
          ++comma_count;
        }
      }
      return comma_count == expected_count;
    }

    void parseStatus(const char *status_line, const size_t status_len){
      unsigned int next_node_id = 0;
      unsigned int next_flags = 0;
      unsigned int next_color = 0;
      unsigned int next_hp = 0;
      unsigned int next_voltage = 0;
      unsigned int next_pid = 0;
      int parsed_length = 0;

      const int matched = sscanf(
        status_line,
        "CC,%u,%x,%x,%x,%x,%x%n",
        &next_node_id,
        &next_flags,
        &next_color,
        &next_hp,
        &next_voltage,
        &next_pid,
        &parsed_length
      );

      if(matched != 6 || static_cast<size_t>(parsed_length) != status_len){
        return;
      }

      if(next_node_id > 0xFFFF || next_flags > 0xFF || next_color > 0xFF ||
         next_hp > 0xFF || next_voltage > 0xFF || next_pid > 0xFF){
        return;
      }

      node_id = static_cast<uint16_t>(next_node_id);
      flags = static_cast<uint8_t>(next_flags);
      destoy = ((flags >> 1) & 0x01) != 0;
      color = static_cast<uint8_t>(next_color);
      hp = static_cast<uint8_t>(next_hp);
      voltage = static_cast<uint8_t>(next_voltage);
      pid = static_cast<uint8_t>(next_pid);
    }

    // void parseHit(char *save_ptr){
    //   const char *node_id_text = strtok_r(nullptr, ",", &save_ptr);
    //   const char *hit_flags_text = strtok_r(nullptr, ",", &save_ptr);
    //   if(node_id_text == nullptr || hit_flags_text == nullptr){
    //     return;
    //   }
    //
    //   uint16_t next_node_id = 0;
    //   uint8_t next_hit_flags = 0;
    //   if(!parseDec16(node_id_text, next_node_id) ||
    //      !parseHex8(hit_flags_text, next_hit_flags)){
    //     return;
    //   }
    //
    //   node_id = next_node_id;
    //   hit_flags = next_hit_flags;
    // }

    void parseLine(){
      if(recv_len == 0){
        return;
      }

      for(int i = static_cast<int>(recv_len) - 3; i >= 0; --i){
        if(recv_line[i] != 'C' || recv_line[i + 1] != 'C' || recv_line[i + 2] != ','){
          continue;
        }

        char *status_line = &recv_line[i];
        const size_t status_len = recv_len - static_cast<size_t>(i);
        if(!hasExpectedCommaCount(status_line, status_len, 6)){
          continue;
        }
        // Serial.println("Parsing status line...");

        parseStatus(status_line, status_len);
        return;
      }
    }
};
