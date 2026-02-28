#pragma once

#include <stdint.h>

#define BUFFER_SIZE 32768

uint8_t _packet_tx_decode[BUFFER_SIZE];
uint8_t _packet_rx_decode[BUFFER_SIZE];
uint8_t _packet_tx_encode[BUFFER_SIZE];
uint8_t _packet_rx_encode[BUFFER_SIZE];
int _packet_tx_len = 0;
int _packet_rx_len = 0;

// 必要な関数
void packet_FrameCallBack();
void packet_PacketCallBack(const uint8_t id, const float *data, const size_t len);

size_t _packet_cobsEncode(const uint8_t *data, size_t length, uint8_t *buffer){
    // assert(data && buffer);

    uint8_t *encode = buffer; // Encoded byte pointer
    uint8_t *codep = encode++; // Output code pointer
    uint8_t code = 1; // Code value

    for (const uint8_t *byte = (const uint8_t *)data; length--; ++byte)
    {
        if (*byte) // Byte not zero, write it
            *encode++ = *byte, ++code;

        if (!*byte || code == 0xff) // Input is zero or block completed, restart
        {
            *codep = code, code = 1, codep = encode;
            if (!*byte || length)
                ++encode;
        }
    }
    *codep = code; // Write final code value

    return (size_t)(encode - buffer);
}

size_t _packet_cobsDecode(const uint8_t *buffer, size_t length, uint8_t *data){
    // assert(buffer && data);

    const uint8_t *byte = buffer; // Encoded input byte pointer
    uint8_t *decode = (uint8_t *)data; // Decoded output byte pointer

    for (uint8_t code = 0xff, block = 0; byte < buffer + length; --block)
    {
        if (block) // Decode block byte
            *decode++ = *byte++;
        else
        {
            block = *byte++;             // Fetch the next block length
            if (block && (code != 0xff)) // Encoded zero, write it unless it's delimiter.
                *decode++ = 0;
            code = block;
            if (!code) // Delimiter code found
                break;
        }
    }

    return (size_t)(decode - (uint8_t *)data);
}

void packet_send(){
  int len = _packet_cobsEncode(_packet_tx_decode, _packet_tx_len, _packet_tx_encode);
  _packet_tx_encode[len] = 0;
  Serial.write(_packet_tx_encode, len+1);
  _packet_tx_len = 0;
}

void _packet_onPacketReceived(const uint8_t* buffer, size_t size){
  int flt_len, uint8_len=-2, i;
  uint8_t id;
  int uint8_count=0;
  float data[64] = {0};
  // 受信処理
  for(i=0;i<size;++i){
    if(uint8_len==-2){
      id = buffer[i];
      uint8_len = -1;
    }else if(uint8_len == -1){
      uint8_len = buffer[i];
      if(uint8_len == 0){
        uint8_len = -2;
      }
    }else if(++uint8_count == uint8_len){
      flt_len = uint8_len/4;
      memcpy((uint8_t*)data, &buffer[i-uint8_len+1], uint8_len);
      packet_PacketCallBack(id, data, flt_len);
      // packet_PacketCallBack(id, (float*)&buffer[i-uint8_len+1], flt_len);
      uint8_len = -2;
      uint8_count = 0;
    }
  }
  // 送信処理
  packet_FrameCallBack();
}

// PCに送るデータを登録
// ID, flaotの配列, 配列の数
bool packet_setUint8(const uint8_t id, const uint8_t *data, const int u8_len){
  int uint8_len = u8_len;
  if (_packet_tx_len + uint8_len >= BUFFER_SIZE)
      return false;
  _packet_tx_decode[_packet_tx_len] = id;
  _packet_tx_decode[_packet_tx_len + 1] = uint8_len;
  memcpy(_packet_tx_decode+_packet_tx_len+2, (uint8_t*)data, uint8_len);
  _packet_tx_len += (uint8_len + 2);
  return true;
}

bool packet_setFloat(const uint8_t id, const float *data, const int flt_len){
  return packet_setUint8(id, (uint8_t*)data, flt_len*4);
}

bool packet_setInt32(const uint8_t id, const int *data, const int int_len){
  return packet_setUint8(id, (uint8_t*)data, int_len*4);
}

// PCに接続
void packet_begin(){
    Serial.begin(9600);
    // while(!Serial){

    // }
}

// PCから受信したデータを解析
void packet_update(){
  while(Serial.available()){
    if(_packet_rx_len == (BUFFER_SIZE-1)){
      _packet_rx_len = 0;
    }
    uint8_t data = Serial.read();
    if(data == 0){
      int size = _packet_cobsDecode(_packet_rx_encode, _packet_rx_len, _packet_rx_decode);
      _packet_onPacketReceived(_packet_rx_decode, size);
      _packet_rx_len = 0;
    }else{
      _packet_rx_encode[_packet_rx_len++] = data;
    }
  }
}