#include <iterator>
#include "wiring.h"
#pragma once

#define STS_CONTROL_DT 0.01f
#define STS_LIMIT_VEL (67 / 60 * 2 * M_PI)

#define STS_PID_P 5

#define STS_TIMEOUT     800  // 応答待ち時間（マイクロ秒）
#define MAX_DATA_LENGTH 2048   // 最大データ長（適宜変更してください）
#define SERVO_ID 0
#define LEN_SERVO 8

struct STS_SERVO {
  float pos=0;
  float vel=0;
  float current=0;
  uint16_t prev_pos = UINT16_MAX;
  int rotate = 0;
};

class STS{
  public:
    unsigned long prev_connect_ts_ = 0;
    bool disable = false;
    bool connect = false;
    uint8_t torque[LEN_SERVO] = {0};
    float ref_pos[LEN_SERVO] = {0};
    float ref_vel[LEN_SERVO] = {0};
    float ref_pos2[LEN_SERVO] = {0};
    float origin[LEN_SERVO] = {M_PI, M_PI+0.900000, -0.9, 0.1, 0, 0, 0, 0};
    STS_SERVO servos[LEN_SERVO];
    PID pos_p1, pos_p2;

    STS(): pos_p1(STS_PID_P, 0, STS_CONTROL_DT),pos_p2(STS_PID_P, 0, STS_CONTROL_DT){}
    void init(){
      Serial2.begin(1000000);  // サーボ通信用として1000000bpsに設定）
      Serial2.addMemoryForRead(&bigserialbuffer_, sizeof(bigserialbuffer_));
      while (!Serial2) {
        // Serialポートが利用可能になるまで待機
      }
      delay(1000);
      prev_connect_ts_ = millis();
      // torque_on();
    }
    void torque_on(){
      for(int i=0;i<LEN_SERVO;++i){
        torque[i] = 0;
         sts_writeReg(servo_ids[i], 55, 1); //eeprom lock
        delay(100);
        if(i <=1)
          sts_writeReg(servo_ids[i], 33, 0); //速度モード
        else
          sts_writeReg(servo_ids[i], 33, 1); //速度モード
      
        delay(100);
        // トルクON
        sts_writeReg(servo_ids[i], 40, 1); // サーボの初期化待ち
        delay(100);
      }
      sts_writePos(0, origin[0]);
      sts_writePos(1, origin[1]);
      sts_writeVel(2, 0);
      sts_writeVel(3, 0);
    }
    void update(){
      static int i = 0;
      bool timeout = (millis() - prev_connect_ts_) > 1000;
      if(timeout){
          connect = false;
      }
      byte index = 56, len = 15;
      // 100Hz
      if(connect){
        for(int i=0;i<2;++i){
          ref_pos2[i] = ref_pos[i]+origin[i];
        }

        for(int i=2;i<4;++i){
          ref_pos2[i] = ref_pos[i];
        }

        sts_syncWritePos(servo_ids, 2, ref_pos2);

        ref_vel[2] = pos_p1.update(ref_pos2[2] - servos[2].pos, 42);
        ref_vel[3] = pos_p2.update(ref_pos2[3] - servos[3].pos, 42);

        if(disable){
          ref_vel[2] = 0;
          ref_vel[3] = 0;
        } 
        sts_syncWriteVel(servo_ids+2, 2, ref_vel+2);
      }else{
        // 10Hz
        if(++i == 10){
          i = 0;
          if(!timeout){
            delay(500);
            torque_on();
            connect = true;
          }
          
          // index = 40;//torque
          // len = 1;//len
          // sts_syncRead(servo_ids, LEN_SERVO, index, len);
          // int enable = 0;
          // for(int n=0;n<LEN_SERVO;++n){
          //   if(torque[n] == 1){
          //     enable++;
          //   }
          // }
          // if(enable == 4){
          //   connect = true;
          // }
        }
      }
      sts_syncRead(servo_ids, LEN_SERVO, index, len);
    }
    void syncReadData() {
      int header_num = 0;
      byte id = 0;
      byte len = 0;
      byte read_data=0;

      // 指定されたバイト数が来るまで待つ
      while(Serial2.available()){
        // オーバーフロー
        if((buffer_data_num + 1) == MAX_DATA_LENGTH){
          buffer_data_num = 0;
        }
        buffer[buffer_data_num++] = (byte)Serial2.read();
      }

      // パケットをパース
      for(int i=0; i<buffer_data_num; ++i){
        read_data = buffer[i];
        if(header_num <= 1){
          if(read_data == 0xff){
            header_num++;
          }else{
            // 捨てる
          }
        }else if(header_num <= 2){
          id = read_data;
          header_num++;
        }else if((buffer_data_num - i) >= read_data){
            len = read_data;
            header_num = 0;
            dispResvData(buffer+i-3, id, len+4);
            i+=len;
        }else{
          // 足りない
          break;
        }
      }
    }
  private:
    uint8_t bigserialbuffer_[MAX_DATA_LENGTH];
    byte servo_ids[LEN_SERVO] = {1, 2, 3, 4};
    byte buffer[MAX_DATA_LENGTH];// 受信データを格納するバッファ
    int buffer_data_num = 0;

    // サーボデータの受信

    void sts_syncRead(byte *ids, byte len_id, byte index, byte len) {
      byte message[64];   // コマンドパケットを作成
      message[0] = 0xFF; // ヘッダ
      message[1] = 0xFF; // ヘッダ
      message[2] = 0xFE;   // サーボID
      message[3] = len_id+4;    // パケットデータ長
      message[4] = 0x82;    // コマンド
      message[5] = index;   // レジスタ先頭番号
      message[6] = len;    // 読み込みバイト数
      memcpy(message+7, ids, len_id);
      message[7+len_id] = sts_calcCkSum(message, 8+len_id); // チェックサム
      Serial2.write(message, 8+len_id);
      Serial2.flush();  //データを送信
      // reset
      buffer_data_num = 0;
    }

    void sts_syncWrite(byte *ids, byte len_id, byte index, byte len, byte *data) {
      byte message[64];   // コマンドパケットを作成
      message[0] = 0xFF; // ヘッダ
      message[1] = 0xFF; // ヘッダ
      message[2] = 0xFE;   // サーボID
      message[3] = (len+1)*len_id+4;    // パケットデータ長
      message[4] = 0x83;    // コマンド
      message[5] = index;    // 先頭アドレス
      message[6] = len;    // 長さ
      int start_addr;
      for(int i=0;i<len_id;++i){
        start_addr = 7 + (len+1)*i;
        message[start_addr + 0] = ids[i];
        memcpy(message + start_addr+ 1, data+i*len, len);
      }
      message[7+(len+1)*len_id] = sts_calcCkSum(message, 8+(len+1)*len_id); // チェックサム
      Serial2.write(message, 8+(len+1)*len_id);
      Serial2.flush();  //データを送信
    }

    void sts_writeVel(byte id, float data) {
      int16_t temp = (int16_t)(data / (2*M_PI) * 4096.f);
      if(temp < 0){
        temp = -temp;
        bitWrite(temp, 15, 1);
      }else{
        bitWrite(temp, 15, 0);
      }
      sts_write(id, 46, 2, (byte*)&temp);
    }

    void sts_writePos(byte id, float data) {
      uint16_t buf[1];   // コマンドパケットを作成
      int buf_int = (data/M_PI/2 * 4096); 
      buf[0] = constrain(buf_int, 0, 4095);
      sts_write(id, 42, 2, (byte*)buf);
    }

    void sts_write(byte id, byte index, byte len, byte *data) {
      byte message[64];   // コマンドパケットを作成
      message[0] = 0xFF; // ヘッダ
      message[1] = 0xFF; // ヘッダ
      message[2] = id;   // サーボID
      message[3] = len+3;    // パケットデータ長
      message[4] = 0x03;    // コマンド
      message[5] = index;    // 先頭アドレス
      memcpy(message+6, data, len);
      message[6+len] = sts_calcCkSum(message, 7+len); // チェックサム
      Serial2.write(message, 7+len);
      Serial2.flush();  //データを送信
    }

    void sts_writeReg(byte id, byte index, byte data) {
      byte buf[1] = {data};
      sts_write(id, index, 1, buf);
    }
    void sts_syncWritePos(byte *ids, byte len_id, float *data){
      uint16_t buf[32];
      uint16_t temp;
      for(int i=0;i<len_id;++i){
        buf[i] = (uint16_t)constrain((data[i] / (2*M_PI) * 4096.f), 0, 4095);
      }
      sts_syncWrite(ids, len_id, 42, 2, (byte*)buf);
    }

    void sts_syncWriteVel(byte *ids, byte len_id, float *data){
      int16_t buf[32];
      int16_t temp;
      for(int i=0;i<len_id;++i){
        temp = (int16_t)(data[i] / (2*M_PI) * 4096.f);
        if(temp < 0){
          temp = -temp;
          bitWrite(temp, 15, 1);
        }else{
          bitWrite(temp, 15, 0);
        }
        buf[i] = temp;
      }
      sts_syncWrite(ids, len_id, 46, 2, (byte*)buf);
    }
    // チェックサムの計算
    byte sts_calcCkSum(byte *arr, int len) {
      byte checksum = 0;
      for (int i = 2; i < len - 1; i++) {
        checksum += arr[i];
      }
      return ~((byte)(checksum & 0xFF)); // チェックサム
    }

    // 受信データの表示
    void dispResvData(byte *data, byte id, byte len) {
      if(data[len-1] != sts_calcCkSum(data, len))
        return;
      if(id > LEN_SERVO){
        return;
      }
      prev_connect_ts_ = millis();
      int i = id - 1;
      if(len > 18){
        uint16_t pos = (uint16_t)(data[6] << 8 | data[5])&0xfff;
        uint16_t vel = (uint16_t)(data[8] << 8 | data[7]);
        uint16_t current = (uint16_t)(data[19] << 8 | data[18]);
    
        // HSBが符号でそれ以外はUINT16
        int vel_i = vel & 0x7fff;
        if(bitRead(vel, 15)){
          vel_i = -vel_i;
        }

        int curr_i = current & 0x7fff;
        if(bitRead(current, 15)){
          curr_i = -curr_i;
        }
        if(servos[i].prev_pos == UINT16_MAX)
          servos[i].prev_pos = pos;
        else if( pos - servos[i].prev_pos > (4096/2))
          servos[i].rotate--;
        else if ( pos - servos[i].prev_pos < -(4096/2))
          servos[i].rotate++;

        servos[i].pos = ((float)pos / 4096.f + servos[i].rotate) *M_PI*2-origin[i];
        servos[i].prev_pos = pos;
        servos[i].vel = (float)vel_i / 4096.f *M_PI*2;
        servos[i].current = (float)curr_i * 0.0065;

      }else if(len > 3){
        torque[i] = data[5]; 
      }
    }
};
