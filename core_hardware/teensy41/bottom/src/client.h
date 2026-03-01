#pragma once

#include <stdlib.h>
#include <stdio.h>

#define PORT_SERIAL Serial4

#define MAX_LEN 2048

byte serial4RXbuffer[MAX_LEN];

class Client{
  public:
    volatile uint8_t hp=100;
    volatile int destoy = 0;
    uint8_t recv_data[MAX_LEN];
    char hp_char[2];
    int recv_len = 0;
    int count = 0;
    Client(){}
    void init(){
      PORT_SERIAL.begin(115200);
      PORT_SERIAL.addMemoryForRead(serial4RXbuffer, MAX_LEN);
    }
    void update(){
      destoy = digitalRead(PIN_DESTROY);
      while(PORT_SERIAL.available()){
        uint8_t now = PORT_SERIAL.read();
        recv_data[recv_len++] = now;
        if(recv_len == MAX_LEN){
          recv_len = 0;
          count = 0;
        }
        if(now == '\r' && count == 0){
          count = 1;
        }else if(now == '\n' && count == 1){
          count = 0;
          if(recv_len == 25){
            hp_char[0] = recv_data[9];
            hp_char[1] = recv_data[10];
            hp = strtol(hp_char, NULL, 16);
            Serial.println((int)hp);
          }
          recv_len = 0;
        }
      }
    }
};

Client client;